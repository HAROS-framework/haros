# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Dict, Iterable, List, NewType, Optional, Set, Tuple, Union

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


@define
class ControlNode:
    id: ControlNodeId
    body: List[PythonStatement] = field(factory=list)
    condition: LogicValue = field(factory=LogicValue.tautology)
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
    nodes: List[ControlNode] = field(factory=list)
    is_asynchronous: bool = False

    @property
    def first_node(self) -> ControlNode:
        return self.nodes[0]

    @property
    def last_node(self) -> ControlNode:
        return self.nodes[-1]


###############################################################################
# Graph Builder
###############################################################################


class MalformedProgramError(Exception):
    @classmethod
    def break_without_loop(cls) -> 'MalformedProgramError':
        return cls('found a break statement outside of a loop')

    @classmethod
    def continue_without_loop(cls) -> 'MalformedProgramError':
        return cls('found a continue statement outside of a loop')

    @classmethod
    def not_branching(cls) -> 'MalformedProgramError':
        return cls('found a branching statement outside of a conditional')

    @classmethod
    def if_after_else(cls) -> 'MalformedProgramError':
        return cls('found a branching statement after an else statement')


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


@define
class ProgramContext:
    current_node: ControlNode

    def must_break(self):
        raise MalformedProgramError.break_without_loop()

    def must_continue(self):
        raise MalformedProgramError.continue_without_loop()

    def new_context(self, node: ControlNode) -> 'ProgramContext':
        return ProgramContext(node)


@define
class LoopContext(ProgramContext):
    guard_node: ControlNode
    future_node: ControlNode

    def must_break(self):
        self.current_node.jump_to(self.future_node)

    def must_continue(self):
        self.current_node.jump_to(self.guard_node)

    def new_context(self, node: ControlNode) -> 'LoopContext':
        return LoopContext(node, self.guard_node, self.future_node)


# ProgramGraphBuilder is ready to `add_statement` after initialisation.
# It keeps a stack of `ProgramContext` upon which it builds new nodes.


@define
class ProgramGraphBuilder:
    _nodes: Dict[ControlNodeId, ControlNode] = field(factory=dict)
    _context_stack: List[ProgramContext] = field(init=False, factory=list)
    _branch_stack: List[BranchingContext] = field(init=False, factory=list)
    _start: ControlNodeId = ControlNodeId(0)
    _asynchronous: bool = False

    def __attrs_post_init__(self):
        node = self._new_node(condition=TRUE)
        self._start = node.id
        context = ProgramContext(node)
        self._context_stack.append(context)

    @property
    def context(self) -> ProgramContext:
        return self._context_stack[-1]

    @property
    def current_node(self) -> ControlNode:
        return self._context_stack[-1].current_node

    @property
    def branch_context(self) -> BranchingContext:
        return self._branch_stack[-1]

    def add_statement(self, statement: PythonStatement):
        context = self.context
        this_node = context.current_node

        if statement.is_while:
            self._build_while(statement)

        elif statement.is_for:
            return False

        else:
            this_node.append(statement)

            if statement.is_break:
                context.must_break()
                self._start_dead_code()

            elif statement.is_continue:
                context.must_continue()
                self._start_dead_code()

            elif statement.is_return or statement.is_raise:
                self._start_dead_code()

            elif statement.is_yield:
                self._asynchronous = True
                self._follow_up_node(switch=True)

            elif statement.is_assert:
                phi = to_condition(statement.condition)
                self._follow_up_node(phi.negate())  # terminal node
                self._follow_up_node(phi, switch=True)

            elif statement.is_if:
                self._build_if(statement)

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

    def build(self) -> ControlFlowGraph:
        cfg = self.cfg
        self.cfg = _cfg_factory()
        return cfg

    def _new_node(
        self,
        condition: Optional[LogicValue] = None,
        origin: Optional[ControlNode] = None,
    ) -> ControlNode:
        uid = ControlNodeId(len(self._nodes))
        if condition is None:
            condition = self.current_node.condition
        node = ControlNode(uid, condition=condition)
        self._nodes[uid] = node
        if origin is not None:
            origin.jump_to(node)
        return node

    def _follow_up_node(
        self,
        phi: Optional[LogicValue] = None,
        *,
        switch: bool = False,
    ) -> ControlNode:
        this_node = self.current_node
        phi = this_node.condition.join(TRUE if phi is None else phi)
        node = self._new_node(condition=phi, origin=this_node)
        if switch:
            self.context.current_node = node
        return node

    def _start_dead_code(self):
        # push a new node that propagates the current condition,
        # but has no incoming links from other nodes
        self.context.current_node = self._new_node()

    def _push_context(self, node: ControlNode) -> ProgramContext:
        new_context = self.context.new_context(node)
        self._context_stack.append(new_context)
        return new_context

    def _start_branching(self) -> BranchingContext:
        guard_node = context.current_node
        future_node = self._new_node(condition=guard_node.condition)
        branch_context = BranchingContext(guard_node, future_node)
        self._branch_stack.append(branch_context)
        return branch_context

    def _stop_branching(self):
        if not self._branch_stack:
            raise MalformedProgramError.not_branching()
        branch_context = self._branch_stack.pop()
        self.context.current_node = branch_context.future_node

    def _build_if(self, statement: PythonStatement):
        self._start_branching()
        self._build_branch(statement.condition, statement.body)
        for branch in statement.elif_branches:
            self._build_branch(branch.condition, branch.body)
        if statement.has_else_branch:
            self._build_branch(None, statement.else_branch)
        self._stop_branching()

    def _build_branch(
        self,
        test: Optional[PythonExpression],
        statements: Iterable[PythonStatement],
    ):
        # create a context to process the new branch
        conditional = self.branch_context
        if test is None:
            phi = conditional.add_else_branch()
        else:
            phi = conditional.add_branch(test)
        branch_node = self._follow_up_node(phi)
        ctx = self._push_context(branch_node)

        # recursively process the statements
        for statement in statements:
            self.add_statement(statement)

        # link to the node that comes after
        self.current_node.jump_to(conditional.future_node)

        # pop the context for the new branch
        assert self.context is ctx, f'unexpected context: {self.context!r}'
        self._context_stack.pop()

    def _start_loop(self) -> LoopContext:
        new_context = LoopContext(self.current_node)
        self._context_stack.append(new_context)
        return new_context

    def _build_while(self, statement: PythonStatement):
        loop_node = self._follow_up_node()


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
