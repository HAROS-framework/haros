// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

// -----------------------------------------------------------------------------
//  Feature Model
// -----------------------------------------------------------------------------

const FeatureModelComponent = {
  template: "#vue-feature-model-component",
  props: {
    model: Object
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
    };
  },
  methods: {
    onCustomEvent(arg1, arg2) {
      this.$emit("custom-event", arg1, arg2);
    }
  }
};
