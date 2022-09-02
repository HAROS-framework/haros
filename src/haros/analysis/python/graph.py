# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Dict, Final, Iterable, List, NewType, Optional, Set, Tuple, Union

from attrs import define, field, frozen

from haros.analysis.python.logic import to_condition
from haros.metamodel.logic import FALSE, TRUE, LogicValue
from haros.parsing.python.ast import (
    PythonAst,
    PythonBooleanLiteral,
    PythonExpression,
    PythonForStatement,
    PythonModule,
    PythonStatement,
    PythonWhileStatement,
    PythonTryStatement,
)

###############################################################################
# Data Structures
###############################################################################

ControlNodeId = NewType('ControlNodeId', int)

ROOT_ID: Final[ControlNodeId] = ControlNodeId(0)


@define
class ControlNode:
    id: ControlNodeId
    body: List[PythonStatement] = field(factory=list)
    condition: LogicValue = field(default=TRUE)
    incoming: Set[ControlNodeId] = field(factory=set)
    outgoing: Set[ControlNodeId] = field(factory=set)

    @property
    def is_empty(self) -> bool:
        return not self.body

    @property
    def is_terminal(self) -> bool:
        return not self.outgoing

    @property
    def is_unreachable(self) -> bool:
        return not self.incoming

    @property
    def is_conditional(self) -> bool:
        return not self.condition.is_true and not self.condition.is_false

    def append(self, statement: PythonStatement):
        self.body.append(statement)

    def jump_to(self, other: 'ControlNode'):
        self.outgoing.add(other.id)
        other.incoming.add(self.id)


@define
class ControlFlowGraph:
    nodes: Dict[ControlNodeId, ControlNode] = field(factory=dict)
    root_id: ControlNodeId = field()
    is_asynchronous: bool = False

    @root_id.validator
    def _check_root_id(self, attribute, value: ControlNodeId):
        if value not in self.nodes:
            raise ValueError(f'unknown node id: {value!r}')

    @property
    def root_node(self) -> ControlNode:
        return self.nodes[self.root_id]

    @classmethod
    def singleton(cls, is_asynchronous: bool = False) -> 'ControlFlowGraph':
        nodes = {ROOT_ID: ControlNode(ROOT_ID)}
        return cls(nodes=nodes, root_id=ROOT_ID, is_asynchronous=is_asynchronous)


###############################################################################
# Graph Builder
###############################################################################


class MalformedProgramError(Exception):
    @classmethod
    def not_looping(cls) -> 'MalformedProgramError':
        return cls('found a loop jump statement outside of a loop')

    @classmethod
    def not_branching(cls) -> 'MalformedProgramError':
        return cls('found a branching statement outside of a conditional')

    @classmethod
    def if_after_else(cls) -> 'MalformedProgramError':
        return cls('found a branching statement after an else statement')


@define
class LoopingContext:
    guard_node: ControlNode
    future_node: ControlNode
    break_condition: LogicValue = field(init=False, default=FALSE)

    def break_from(self, node: ControlNode):
        node.jump_to(self.future_node)
        phi = node.condition.assume(self.guard_node.condition)
        self.break_condition = self.break_condition.disjoin(phi)

    def continue_from(self, node: ControlNode):
        node.jump_to(self.guard_node)


@define
class BranchingContext:
    guard_node: ControlNode
    future_node: ControlNode
    condition: LogicValue = field(init=False, default=FALSE)
    previous: LogicValue = field(init=False, default=TRUE)

    def add_branch(self, test: PythonExpression) -> LogicValue:
        return self._add_branch(to_condition(test))

    def add_else_branch(self) -> LogicValue:
        return self._add_branch(TRUE)

    def _add_branch(self, phi: LogicValue) -> LogicValue:
        if self.condition.is_true:
            raise MalformedProgramError.if_after_else()
        # negate the condition of the current (now previous) branch
        psi = self.condition.negate()
        # join with conditions from past branches
        self.previous = self.previous.join(psi)
        # condition evaluated at the new branch
        self.condition = phi
        # return full condition
        return self.previous.join(self.condition)


# ProgramGraphBuilder is ready to `add_statement` after initialisation.
# It keeps a stack of `LoopingContext` and a stack of `BranchingContext`
# which it uses to build and link nodes.


@define
class ProgramGraphBuilder:
    cfg: ControlFlowGraph = field(factory=ControlFlowGraph.singleton)
    current_id: ControlNodeId = ROOT_ID
    _loop_stack: List[LoopingContext] = field(factory=list)
    _branch_stack: List[BranchingContext] = field(factory=list)

    @classmethod
    def from_scratch(cls):
        cfg = ControlFlowGraph.singleton()
        return cls(cfg=cfg, current_id=cfg.root_id)

    @property
    def current_node(self) -> ControlNode:
        return self.cfg.nodes[self.current_id]

    @current_node.setter
    def current_node(self, node: ControlNode):
        self.current_id = node.id

    @property
    def loop_context(self) -> LoopingContext:
        if not self._loop_stack:
            raise MalformedProgramError.not_looping()
        return self._loop_stack[-1]

    @property
    def branch_context(self) -> BranchingContext:
        if not self._branch_stack:
            raise MalformedProgramError.not_branching()
        return self._branch_stack[-1]

    def add_statement(self, statement: PythonStatement):
        this_node = self.current_node

        if statement.is_while:
            guard_node = self._follow_up_node(switch=True)
            self._start_looping(guard_node)
            self._build_loop(statement.condition, statement.body)
            self._build_loop_else(statement.else_branch)
            self._stop_looping()

        elif statement.is_for:
            return False

        else:
            this_node.append(statement)

            if statement.is_break:
                self.loop_context.break_from(this_node)
                self.start_dead_code()

            elif statement.is_continue:
                self.loop_context.continue_from(this_node)
                self.start_dead_code()

            elif statement.is_return or statement.is_raise:
                self.start_dead_code()

            elif statement.is_yield:
                self.cfg.is_asynchronous = True
                self._follow_up_node(switch=True)

            elif statement.is_assert:
                phi = to_condition(statement.condition)
                self._follow_up_node(phi.negate())  # terminal node
                self._follow_up_node(phi, switch=True)

            elif statement.is_if:
                self._start_branching(this_node)
                self._build_branch(statement.condition, statement.body)
                for branch in statement.elif_branches:
                    self._build_branch(branch.condition, branch.body)
                self._build_branch(None, statement.else_branch)
                self._stop_branching()

            elif statement.is_try:
                return False
            elif statement.is_match:
                return False
            elif statement.is_with:
                return False
            elif statement.is_function_def:
                return False
            elif statement.is_class_def:
                return
        return True

    def start_dead_code(self):
        # push a new node that propagates the current condition,
        # but has no incoming links from other nodes
        self._new_node(phi=self.current_node.condition, switch=True)

    def _new_node(
        self,
        phi: LogicValue,
        *,
        origin: Optional[ControlNode] = None,
        switch: bool = False,
    ) -> ControlNode:
        uid = ControlNodeId(len(self.cfg.nodes))
        node = ControlNode(uid, condition=phi)
        self.cfg.nodes[uid] = node
        if origin is not None:
            origin.jump_to(node)
        if switch:
            self.current_id = node.id
        return node

    def _follow_up_node(self, phi: LogicValue = TRUE, *, switch: bool = False) -> ControlNode:
        this_node = self.current_node
        phi = this_node.condition.join(phi)
        return self._new_node(phi, origin=this_node, switch=switch)

    def _push_context(self, node: ControlNode) -> ProgramContext:
        new_context = self.context.new_context(node)
        self._context_stack.append(new_context)
        return new_context

    def _start_branching(self, guard_node: ControlNode) -> BranchingContext:
        future_node = self._new_node(guard_node.condition)
        context = BranchingContext(guard_node, future_node)
        self._branch_stack.append(context)
        return context

    def _stop_branching(self):
        if not self._branch_stack:
            raise MalformedProgramError.not_branching()
        context = self._branch_stack.pop()
        self.current_id = context.future_node.id

    def _build_branch(
        self,
        test: Optional[PythonExpression],
        statements: Iterable[PythonStatement],
    ):
        # create and move to a new branch
        conditional = self.branch_context
        self.current_id = conditional.guard_node.id
        if test is None:
            phi = conditional.add_else_branch()
        else:
            phi = conditional.add_branch(test)
        branch_node = self._follow_up_node(phi, switch=True)

        # recursively process the statements
        for statement in statements:
            self.add_statement(statement)

        # link to the node that comes after
        self.current_node.jump_to(conditional.future_node)

    def _start_looping(self, guard_node: ControlNode) -> LoopingContext:
        future_node = self._new_node(guard_node.condition)
        context = LoopingContext(guard_node, future_node)
        self._loop_stack.append(context)
        return context

    def _stop_looping(self):
        if not self._loop_stack:
            raise MalformedProgramError.not_looping()
        context = self._loop_stack.pop()
        self.current_id = context.future_node.id

    def _build_loop(self, test: PythonExpression, statements: Iterable[PythonStatement]):
        # very similar to `_build_branch`
        # create and move to a new branch
        phi = to_condition(test)
        self._follow_up_node(phi, switch=True)

        # recursively process the statements
        for statement in statements:
            self.add_statement(statement)

        # link back to the loop guard node
        self.current_node.jump_to(self.loop_context.guard_node)

    def _build_loop_else(self, statements: Iterable[PythonStatement]):
        loop = self.loop_context
        self.current_id = loop.guard_node.id

        # create and move to a new branch
        phi = loop.break_condition.negate()
        self._follow_up_node(phi, switch=True)

        # recursively process the statements
        for statement in statements:
            self.add_statement(statement)

        # link to the node that comes after
        self.current_node.jump_to(loop.future_node)


###############################################################################
# Interface
###############################################################################


def from_ast(ast: PythonAst) -> Scope:
    if not ast.is_statement:
        raise TypeError(f'expected a statement, got {ast!r}')
    if ast.is_module:
        return from_module(ast)
    raise TypeError(f'unexpected tree node: {ast!r}')


def from_module(module: PythonModule) -> Scope:
    return
