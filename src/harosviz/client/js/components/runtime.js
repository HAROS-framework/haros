// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

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
