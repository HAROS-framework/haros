// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

const DashboardPage = {
  template: "#vue-dashboard-page",
  props: {
    title: {
      type: String,
      required: true
    },
    text: {
      type: String,
      required: true
    },
    compact: Boolean
  },
  data() {
    return {
      source: [
        {name: 'Packages', quantity: 12},
      ],
      nodes: [
        {name: 'Models', quantity: 12},
      ],
      runtime: [
        {name: 'Models', quantity: 12},
      ],
      issues: [
        {name: 'Total', quantity: 100},
      ],
    };
  },
  methods: {
    onCustomEvent(arg1, arg2) {
      this.$emit("custom-event", arg1, arg2);
    }
  },

  mounted() {
    d3.select(this.$refs.chartContainer1).append(window.newChart);
    d3.select(this.$refs.chartContainer2).append(window.newChart);
  }
};
