// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

/*

Some example graph visualizations to keep an eye on.

# Hierarchical Edge Bundling
https://observablehq.com/@d3/hierarchical-edge-bundling/2?intent=fork

Circular layout, ideal to show node-to-node relations.
Colored edges indicate the direction of the arrow.

# Arc Diagram
https://observablehq.com/@d3/arc-diagram

Another useful representation of node-to-node connections.
Maybe not as good as the radial representation, but provides additional space
for node information, and this example also has a neat sorting feature.

# Disjoint force-directed graph
https://observablehq.com/@d3/disjoint-force-directed-graph/2?intent=fork

Standard force graph, but this one accounts for disjoint nodes, which is a
must-have in incomplete models.
This type of graph can include all kinds of resources (topics, etc.).
Other examples on the web include circle nodes of diffent sizes, which might
be interesting to add.

# Bump Charts
https://observablehq.com/d/aa2ed4c606d995c0

This example is not what we actually want, but might provide a code basis for
another interesting visualization, with nodes as boxes on one side and other
resources (topics, etc.) as boxes on the other side.

It might be easier to navigate than a standard force graph, and each box would
highlight what is connected to it. Reusing ideas from previous examples, links
should use a color gradient to indicate the direction of information flow.
For example, nodes would be grey and topics colored. Advertise links fade from
grey to color, and subscribe links do the opposite.

*/

const ICON_TRUE = "\u2713";
const ICON_FALSE = "\u2716"; // "\u00d7";
const ICON_MAYBE = "";

const FEATURE_TYPE_ROSLAUNCH = 1;
const FEATURE_TYPE_ARGUMENT = 2;
const FEATURE_TYPE_VALUE = 3;

// -----------------------------------------------------------------------------
//  Feature Model
// -----------------------------------------------------------------------------

UI.FeatureTreeItem = {
  name: 'TreeItem', // necessary for self-reference
  template: "#vue-feature-tree-item",

  props: {
    model: Object,
    depth: Number,
    isParentSelected: Boolean,
  },

  data() {
    return {
      isCollapsed: false
    }
  },

  watch: {
    isParentSelected(newValue, oldValue) {
      switch (this.model.type) {
        case ARG_TYPE:
          return this.propagateRosLaunchSelectionDown(newValue, oldValue);
        case VALUE_TYPE:
          return this.propagateArgSelectionDown(newValue, oldValue);
      }
      if (newValue === false) {
        // deselect this
        this.model.selected = false;
      } else if (oldValue === false) {
        // liberate selection
        if (this.model.selected === false) {
          this.model.selected = null;
        }
      }
    }
  },

  computed: {
    hasChildren() {
      return this.model.children && this.model.children.length
    },

    selectionStatus() {
      return this.model.selected
        ? ICON_TRUE
        : (this.model.selected === false
        ? ICON_FALSE
        : ICON_MAYBE);
    }
  },

  methods: {
    toggle() {
      if (this.hasChildren) {
        this.isCollapsed = !this.isCollapsed
      }
    },

    onSelect() {
      // skip the root
      if (this.depth === 0) { return; }
      switch (this.model.type) {
        case ROSLAUNCH_TYPE:
          return this.onSelectRosLaunch();
        case ARG_TYPE:
          return this.onSelectArg();
        case VALUE_TYPE:
          return this.onSelectValue();
      }
      // if (this.model.selected) {
      //   this.model.selected = false;
      // } else if (this.model.selected === false) {
      //   this.model.selected = null;
      // } else {
      //   console.log("Selected feature:", this.model.name);
      //   this.model.selected = true;
      // }
    },

    onSelectRosLaunch() {
      console.assert(this.model.type === FEATURE_TYPE_ROSLAUNCH);
      if (this.model.implicit) {
        this.model.implicit = false;
      } else {
        this.model.implicit = false;
        if (this.model.selected) {
          this.model.selected = false;
        } else if (this.model.selected === false) {
          this.model.selected = null;
        } else {
          this.model.selected = true;
        }
        // propagate roslaunch selection to siblings (conflict handling)
        this.$emit("roslaunch-selected", this.model.id, this.model.selected);
        // propagation downwards is handled with property watchers
      }
    },

    onSelectArgument() {
      if (this.model.implicit) {
        this.model.implicit = false;
        if (this.model.selected) {
          this.propagateArgSelectionDown();
        }
      } else {
        if (this.model.selected === false) {
          // console.assert(this.parent.selected === false);
          return;
        }
        if (this.model.selected) {
          // console.assert(this.parent.selected);
          this.model.selected = null;
          this.model.implicit = false;
        } else {
          // console.assert(this.parent.selected !== false);
          // propagate up to the parent and down to the siblings
          this.propagateArgSelectionUp();
          // change this selection
          this.model.selected = true;
          this.model.implicit = false;
        }
        this.propagateArgSelectionDown();
      }
    },

    onSelectValue() {

    },

    propagateRosLaunchSelectionDown(exclude) {
      const children = this.model.children;
      if (!children) { return; }
      const v = this.model.selected;
      if (v) {
        for (const child of children) {
          // skip those that are already selected
          if (child.selected) { continue; }
          // propagate null`to deselect all values
          child.selected = null;
          child.implicit = true;
          const ok = this.propagateArgSelection(d);
          // propagating null always works
          console.assert(ok);
        }
      } else {
        for (const d of children) {
          // propagate `null` to deselect all values
          d.ui.selected = null;
          d.data.selected = null;
          const ok = this.propagateArgSelection(d);
          // propagating `null` always works
          console.assert(ok);
          d.ui.selected = v;
          d.data.selected = v;
        }
      }
    },

    propagateArgSelectionUp() {
      this.$emit("argument-selected", this.model.id);
    },

    propagateArgSelectionDown() {

    },

    onChildRosLaunchSelected(id) {
      // const files = this.model.children;
      // for (const file of files) {
      //   // update all, including self
      //   this.checkFileConflicts(file);
      // }
    },

    onChildArgumentSelected(id) {
      console.assert(this.model.type === FEATURE_TYPE_ROSLAUNCH, `type: ${this.model.type}`);
      this.model.selected = true;
      this.model.implicit = false;
      this.propagateRosLaunchSelectionDown([id]);
    },

    // addChild() {
    //   this.model.children.push({
    //     name: 'new stuff'
    //   })
    // }
  }
};


UI.FeatureTreeView = {
  template: "#vue-feature-tree-view",
  components: {
    FeatureTreeItem: UI.FeatureTreeItem
  },
  props: {
    tree: {
      type: Object,
      required: true,
    }
  }
};


const FeatureModelComponent = {
  template: "#vue-feature-model-component",
  components: {
    FeatureTreeView: UI.FeatureTreeView
  },
  props: {
    model: Object,
    tree: Object,
  },
  data() {},
  methods: {}
};


// -----------------------------------------------------------------------------
//  Computation Graph
// -----------------------------------------------------------------------------

const ComputationGraphComponent = {
  template: "#vue-computation-graph-component",

  props: {
    model: Object
  },

  data() {
    return {
      zoom: 1.0,
      svgWidth: 320,
      svgHeight: 180,
      selectedNode: null,
    };
  },

  watch: {
    model: {
      flush: "post",
      handler(_newModel, _oldModel) {
        this.insertModelElement();
      }
    },

    zoom(newValue, _oldValue) {
      // Specify the dimensions of the graph.
      const zoomFactor = 1.0 / newValue;
      this.svgWidth = 320 * zoomFactor;
      this.svgHeight = 180 * zoomFactor;
      const svg = d3.select(this.$refs.computationGraphContainer)
        .selectAll("svg");
      svg.attr("viewBox", [
        -this.svgWidth / 2,     // min X
        -this.svgHeight / 2,    // min Y
        this.svgWidth,          // width
        this.svgHeight          // height
      ]);
    }
  },

  methods: {
    buildModelElement() {
      const self = this;

      // Specify the color scale.
      const color = d3.scaleOrdinal(d3.schemeCategory10);

      // The force simulation mutates links and nodes, so create a copy
      // so that re-evaluating this cell produces the same result.
      const links = this.model.links.map(d => ({...d}));
      const nodes = this.model.nodes.map(d => ({...d}));

      // Create a simulation with several forces.
      const simulation = d3.forceSimulation(nodes)
        .force("link", d3.forceLink(links).id(d => d.id))
        .force("charge", d3.forceManyBody())
        .force("x", d3.forceX())
        .force("y", d3.forceY());

      // Create the SVG container.
      const svg = d3.create("svg")
        .attr("preserveAspectRatio", "xMidYMid meet")
        .attr("viewBox", [
          -this.svgWidth / 2,     // min X
          -this.svgHeight / 2,    // min Y
          this.svgWidth,          // width
          this.svgHeight          // height
        ]);

      // Add a line for each link, and a circle for each node.
      const link = svg.append("g")
        .attr("stroke", "#999")
        .attr("stroke-opacity", 0.6)
        .selectAll("line")
        .data(links)
        .join("line")
        .attr("stroke-width", d => Math.sqrt(d.value));

      const node = svg.append("g")
        .attr("stroke", "#fff")
        .attr("stroke-width", 1.5)
        .selectAll("circle")
        .data(nodes)
        .join("circle")
        .attr("r", 10)
        .attr("stroke-dasharray", d => d.condition == null ? "" : "2 1")
        .attr("fill", d => color(d.group));

      node.append("title")
          .text(d => d.id);

      // Add a drag behavior.
      node.call(d3.drag()
            .on("start", dragstarted)
            .on("drag", dragged)
            .on("end", dragended));

      function nodeClickHandler(el, datum) {
        const i = datum.index;
        const prev = self.selectedNode;
        if (prev != null) {
          const k = prev.index;
          if (k === i) {
            return;
          }
          d3.select(node.nodes()[k]).attr("fill", d => color(d.group));
        }
        self.onNodeSelected(datum);
        d3.select(el).attr("fill", "#c9c9c9");
      }

      node.on("click", (e, d) => {
        e.stopPropagation();
        // Drag started event is fired before click, so the click handler
        // has already run when we get here. This only has to stop
        // propagation, to prevent the root SVG click handler from firing
        // also, which would nullify the node selection.
        // nodeClickHandler(e.target, d);
      });

      svg.on("click", e => {
        e.stopPropagation();
        const prev = self.selectedNode;
        if (prev != null) {
          const k = prev.index;
          d3.select(node.nodes()[k]).attr("fill", d => color(d.group));
          self.onNodeSelected(null);
        }
      });

      // Set the position attributes of links and nodes each time the simulation ticks.
      simulation.on("tick", () => {
        link
            .attr("x1", d => d.source.x)
            .attr("y1", d => d.source.y)
            .attr("x2", d => d.target.x)
            .attr("y2", d => d.target.y);
        node
            .attr("cx", d => d.x)
            .attr("cy", d => d.y);
      });

      // Reheat the simulation when drag starts, and fix the subject position.
      function dragstarted(event) {
        if (!event.active) simulation.alphaTarget(0.3).restart();
        event.subject.fx = event.subject.x;
        event.subject.fy = event.subject.y;
        const e = event.sourceEvent;
        e.stopPropagation();
        nodeClickHandler(e.target, event.subject);
      }

      // Update the subject (dragged node) position during drag.
      function dragged(event) {
        event.subject.fx = event.x;
        event.subject.fy = event.y;
      }

      // Restore the target alpha so the simulation cools after dragging ends.
      // Unfix the subject position now that it’s no longer being dragged.
      function dragended(event) {
        if (!event.active) simulation.alphaTarget(0);
        event.subject.fx = null;
        event.subject.fy = null;
      }

      return svg.node();
    },

    insertModelElement() {
      const container = d3.select(this.$refs.computationGraphContainer);
      container.selectAll("*").remove();
      if (this.model != null) {
        container.append(this.buildModelElement);
      }
    },

    onZoomIn() {
      this.zoom *= 2.0;
    },

    onZoomOut() {
      this.zoom /= 2.0;
    },

    onNodeSelected(data) {
      this.selectedNode = data;
    }
  },

  mounted() {
    this.insertModelElement();
  }
};


// -----------------------------------------------------------------------------
//  Main Page Controller
// -----------------------------------------------------------------------------

const RuntimePage = {
  template: "#vue-runtime-page",

  components: {
    FeatureModelComponent,
    ComputationGraphComponent,
  },

  props: {},

  data() {
    return {
      fm: null,
      cg: this.exampleCG(),
      tree: this.exampleFeatureTree(),
    };
  },

  methods: {
    exampleFeatureTree() {
      return {
        id: "root",
        name: 'Feature Model',
        children: [
          {
            id: "f.1",
            name: 'launch file',
            type: FEATURE_TYPE_ROSLAUNCH,
          },
          {
            id: "f.2",
            name: 'launch file 2',
            type: FEATURE_TYPE_ROSLAUNCH,
            children: [
              {
                id: "a.2.1",
                name: 'argument 1',
                type: FEATURE_TYPE_ARGUMENT,
                children: [
                  {
                    id: "v.2.1.1",
                    name: 'value 1',
                    type: FEATURE_TYPE_VALUE,
                  },
                  {
                    id: "v.2.1.2",
                    name: 'value 2',
                    type: FEATURE_TYPE_VALUE,
                  }
                ]
              },
              {
                id: "a.2.2",
                name: 'argument 2',
                type: FEATURE_TYPE_ARGUMENT,
              },
            ]
          }
        ]
      };
    },

    exampleCG() {
      return {
        nodes: [
          {
            id: "package_1/node_1",
            group: "package_1",
            condition: null,
          },
          {
            id: "package_1/node_2",
            group: "package_1",
            condition: {name: "z"}
          },
          {
            id: "package_2/node_1",
            group: "package_2",
            condition: ["and", {name: "x"}, {name: "y"}]
          }
        ],
        links: [
          {
            source: "package_1/node_1",
            target: "package_1/node_2",
            value: 2
          },
          {
            source: "package_1/node_1",
            target: "package_2/node_1",
            value: 2
          }
        ],
      };
    }
  }
};
