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

const ComputationGraphComponent = {
  template: "#vue-computation-graph-component",
  props: {
    model: Object
  },
  data() {},
  methods: {}
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
      cg: null,
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
    onCustomEvent(arg1, arg2) {
      this.$emit("custom-event", arg1, arg2);
    }
  }
};
