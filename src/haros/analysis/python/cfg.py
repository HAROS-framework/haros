# SPDX-License-Identifier: MIT
# Copyright © 2022 André Santos

###############################################################################
# Imports
###############################################################################

from typing import Any, Dict, Final, Iterable, List, NewType, Optional, Set, Tuple, Union

from attrs import define, field, frozen

from haros.analysis.python import logic as PythonLogic
from haros.errors import ControlFlowError
from haros.metamodel.logic import FALSE, TRUE, LogicValue
from haros.parsing.python.ast import (
    PythonAst,
    PythonAstNodeId,
    PythonBinaryOperator,
    PythonBooleanLiteral,
    PythonExpression,
    PythonForStatement,
    PythonModule,
    PythonStatement,
    PythonTryStatement,
    PythonTupleLiteral,
    PythonWhileStatement,
)

###############################################################################
# Constants
###############################################################################

ControlNodeId = NewType('ControlNodeId', int)

ROOT_ID: Final[ControlNodeId] = ControlNodeId(0)

MAIN: Final[str] = '__main__'

###############################################################################
# Data Structures
###############################################################################


@frozen
class ControlJump:
    node: ControlNodeId
    condition: LogicValue

    def pretty(self) -> str:
        return str(self)

    def __str__(self) -> str:
        return f'{self.node} ({self.condition})'


@frozen
class ControlNode:
    id: ControlNodeId
    body: List[PythonStatement] = field(factory=list, eq=False, hash=False)
    condition: LogicValue = field(default=TRUE, eq=False, hash=False)
    incoming: Dict[ControlNodeId, LogicValue] = field(factory=dict, eq=False, hash=False)
    outgoing: Dict[ControlNodeId, LogicValue] = field(factory=dict, eq=False, hash=False)

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

    def jump_to(self, other: 'ControlNode', condition: LogicValue = TRUE):
        self.outgoing[other.id] = condition
        other.incoming[self.id] = condition

    def pretty(self) -> str:
        lines = [
            f'# Node {self.id}',
            f'condition:\n  {self.condition}',
            f'incoming:\n  {list(map(str, self.incoming))}',
            f'outgoing:\n  {list(map(str, self.outgoing))}',
        ]
        if self.body:
            lines.append('body:')
            for statement in self.body:
                lines.append(statement.pretty(indent=1, step=2))
        return '\n'.join(lines)


@frozen
class ControlFlowGraph:
    name: str
    root_id: ControlNodeId = field()
    nodes: Dict[ControlNodeId, ControlNode] = field(factory=dict)
    asynchronous: bool = False

    @root_id.validator
    def _check_root_id(self, attribute, value: ControlNodeId):
        if value not in self.nodes:
            raise ValueError(f'unknown node id: {value!r}')

    @property
    def root_node(self) -> ControlNode:
        return self.nodes[self.root_id]

    @classmethod
    def singleton(cls, name: str = MAIN, asynchronous: bool = False) -> 'ControlFlowGraph':
        nodes = {ROOT_ID: ControlNode(ROOT_ID)}
        return cls(name, ROOT_ID, nodes=nodes, asynchronous=asynchronous)

    def pretty(self) -> str:
        header = f'# async {self.name}' if self.asynchronous else f'# {self.name}'
        root = f'root: {self.root_id}'
        lines = [header, root, 'graph:']
        lines.append(f'  {self.root_id} -> {list(map(str, self.root_node.outgoing))}')
        unreachable = []
        for uid, node in self.nodes.items():
            if uid == self.root_id:
                continue
            if node.incoming:
                lines.append(f'  {uid} -> {list(map(str, node.outgoing))}')
            else:
                unreachable.append(f'  {uid} -> {list(map(str, node.outgoing))}')
        if unreachable:
            lines.append('dead code:')
            lines.extend(unreachable)
        return '\n'.join(lines)


###############################################################################
# Graph Builder
###############################################################################

# TODO change loops to have an initial guard node and also a second guard
# (or loop back node) at the end. This helps with unrolling and helps
# evaluate conditions with data flow, and defining jumps when there
# are breaks involved.


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
    branch_leaves: List[ControlNode] = field(init=False, factory=list)
    condition: LogicValue = field(init=False, default=FALSE)
    previous: LogicValue = field(init=False, default=TRUE)
    terminal_branch: bool = False
    got_else_branch: bool = False

    def add_branch(self, phi: LogicValue) -> LogicValue:
        if self.got_else_branch:
            raise ControlFlowError.if_after_else()
        # reset terminal flag
        self.terminal_branch = False
        # negate the condition of the current (now previous) branch
        psi = self.condition.negate()
        # join with conditions from past branches
        self.previous = self.previous.join(psi)
        # condition evaluated at the new branch
        self.condition = phi
        # return full condition
        return self.previous.join(phi)

    def add_else_branch(self) -> LogicValue:
        phi = self.add_branch(TRUE)
        self.got_else_branch = True
        return phi

    def add_branch_leaf(self, node: ControlNode):
        if not self.terminal_branch:
            self.branch_leaves.append(node)

    def close_branches(self, future_node: ControlNode):
        for node in self.branch_leaves:
            node.jump_to(future_node)


# ControlFlowGraphBuilder is ready to `add_statement` after initialisation.
# It keeps a stack of `LoopingContext` and a stack of `BranchingContext`
# which it uses to build and link nodes.


@define
class BasicControlFlowGraphBuilder:
    name: str
    asynchronous: bool = False
    root_id: ControlNodeId = ROOT_ID
    current_id: ControlNodeId = ROOT_ID
    nodes: Dict[ControlNodeId, ControlNode] = field(factory=dict)
    _node_id_counter: int = 0
    _loop_stack: List[LoopingContext] = field(factory=list, eq=False, hash=False)
    _branch_stack: List[BranchingContext] = field(factory=list, eq=False, hash=False)

    @classmethod
    def from_scratch(cls, name: str = MAIN, asynchronous: bool = False):
        builder = cls(name, asynchronous=asynchronous)
        root = builder.new_node()
        builder.root_id = root.id
        return builder

    @property
    def current_node(self) -> ControlNode:
        return self.nodes[self.current_id]

    @current_node.setter
    def current_node(self, node: ControlNode):
        self.current_id = node.id

    @property
    def loop_context(self) -> LoopingContext:
        if not self._loop_stack:
            raise ControlFlowError.not_looping()
        return self._loop_stack[-1]

    @property
    def branch_context(self) -> BranchingContext:
        if not self._branch_stack:
            raise ControlFlowError.not_branching()
        return self._branch_stack[-1]

    def add_statement(self, statement: PythonStatement):
        self.current_node.append(statement)

    def clean_up(self):
        # removes useless nodes from the graph
        for node in list(self.nodes.values()):
            if node.id == self.root_id or len(node.body) > 0:
                continue
            if node.is_unreachable:
                del self.nodes[node.id]

    def jump_to_new_node(self, phi: LogicValue = TRUE) -> ControlNode:
        # phi is the jump condition only
        this_node = self.current_node
        condition = this_node.condition.join(phi)
        node = self.new_node(condition)
        this_node.jump_to(node, condition=phi)
        self.current_id = node.id
        return node

    def new_node(self, phi: LogicValue = TRUE) -> ControlNode:
        # phi is the full condition of the node
        uid = ControlNodeId(self._node_id_counter)
        while uid in self.nodes:
            self._node_id_counter += 1
            uid = ControlNodeId(self._node_id_counter)
        node = ControlNode(uid, condition=phi)
        self.nodes[uid] = node
        return node

    def switch_to(self, node_or_id: Union[ControlNodeId, ControlNode]):
        uid = node_or_id
        if isinstance(node_or_id, ControlNode):
            uid = node_or_id.id
        # ensure that the node is registered
        self.nodes[uid]
        self.current_id = uid

    def start_dead_code(self):
        # push a new node that has no incoming links from other nodes
        self.current_node = self.new_node(FALSE)
        # dead code should not link beyond the current if statement (if any)
        if self._branch_stack:
            self._branch_stack[-1].terminal_branch = True

    def start_branching(self):
        context = BranchingContext(self.current_node)
        self._branch_stack.append(context)

    def stop_branching(self):
        if not self._branch_stack:
            raise ControlFlowError.not_branching()
        context = self._branch_stack.pop()
        future_node = self.new_node(phi=context.guard_node.condition)
        # link dangling branches to the node that comes after
        context.close_branches(future_node)
        self.current_id = future_node.id

    def add_branch(self, phi: LogicValue) -> ControlNode:
        conditional = self.branch_context
        self.current_id = conditional.guard_node.id
        # phi is the condition as written in the if statement
        # psi is the full branch condition (negates previous branches)
        psi = conditional.add_branch(phi)
        return self.jump_to_new_node(psi)

    def add_else_branch(self) -> ControlNode:
        conditional = self.branch_context
        self.current_id = conditional.guard_node.id
        # psi is the full branch condition (negates previous branches)
        psi = conditional.add_else_branch()
        return self.jump_to_new_node(psi)

    def close_branch(self):
        conditional = self.branch_context
        # register dangling branch to link to a new node later
        conditional.add_branch_leaf(self.current_node)

    def start_looping(self) -> ControlNode:
        guard_node = self.jump_to_new_node()
        future_node = self.new_node(guard_node.condition)
        context = LoopingContext(guard_node, future_node)
        self._loop_stack.append(context)
        return guard_node

    def stop_looping(self) -> ControlNode:
        if not self._loop_stack:
            raise ControlFlowError.not_looping()
        context = self._loop_stack.pop()
        # link to the node that comes after
        self.current_node.jump_to(context.future_node)
        self.current_id = context.future_node.id
        return context.future_node

    def close_loop(self):
        # link back to the loop guard node
        self.current_node.jump_to(self.loop_context.guard_node)

    def add_loop_else_branch(self) -> ControlNode:
        loop = self.loop_context
        self.current_id = loop.guard_node.id
        # create and move to a new branch
        phi = loop.break_condition.negate()
        return self.jump_to_new_node(phi=phi)

    def build(self) -> ControlFlowGraph:
        return ControlFlowGraph(
            self.name,
            root_id=self.root_id,
            nodes=dict(self.nodes),
            asynchronous=self.asynchronous,
        )
