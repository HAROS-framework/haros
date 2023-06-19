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

// -----------------------------------------------------------------------------
//  Feature Model
// -----------------------------------------------------------------------------

const FeatureModelComponent = {
  template: "#vue-feature-model-component",
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

function buildComputationGraph(data) {
  // Specify the dimensions of the chart.
  const width = 480;
  const height = 360;
  const zoomLevel = 1.0;
  const zoomFactor = 1.0 / zoomLevel;
  const zoomWidth = width * zoomFactor;
  const zoomHeight = height * zoomFactor;

  // Specify the color scale.
  const color = d3.scaleOrdinal(d3.schemeCategory10);

  // The force simulation mutates links and nodes, so create a copy
  // so that re-evaluating this cell produces the same result.
  const links = data.links.map(d => ({...d}));
  const nodes = data.nodes.map(d => ({...d}));

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
        -zoomWidth / 2,   // min X
        -zoomHeight / 2,  // min Y
        zoomWidth,        // width
        zoomHeight        // height
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
      .attr("r", 5)
      .attr("fill", d => color(d.group));

  node.append("title")
      .text(d => d.id);

  // Add a drag behavior.
  node.call(d3.drag()
        .on("start", dragstarted)
        .on("drag", dragged)
        .on("end", dragended));

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

  // When this cell is re-run, stop the previous simulation. (This doesn’t
  // really matter since the target alpha is zero and the simulation will
  // stop naturally, but it’s a good practice.)
  // invalidation.then(() => simulation.stop());

  return svg.node();
}


const ComputationGraphComponent = {
  template: "#vue-computation-graph-component",

  props: {
    model: Object
  },

  data() {},

  watch: {
    model: {
      flush: "post",
      handler(_newModel, _oldModel) {
        this.insertModelElement();
      }
    }
  },

  methods: {
    buildModelElement() {
      return buildComputationGraph(this.model);
    },

    insertModelElement() {
      const container = d3.select(this.$refs.computationGraphContainer);
      container.selectAll("*").remove();
      if (this.model != null) {
        container.append(this.buildModelElement);
      }
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
      tree: {
        name: 'Feature Model',
        children: [
          { name: 'launch file' },
          {
            name: 'launch file 2',
            children: [
              {
                name: 'argument 1',
                children: [{ name: 'value 1' }, { name: 'value 2' }]
              },
              { name: 'argument 2' },
            ]
          }
        ]
      }
    };
  },
  methods: {
    exampleCG() {
      return {
        nodes: [
          {
            id: "package_1/node_1",
            group: "package_1",
            // radius: 2,
          },
          {
            id: "package_1/node_2",
            group: "package_1",
            // radius: 2,
          },
          {
            id: "package_1/node_3",
            group: "package_1",
            // radius: 2,
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
            target: "package_1/node_3",
            value: 2
          }
        ],
      };
    }
  }
};
