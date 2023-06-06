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
        {name: 'Files', quantity: 123},
        {name: 'Nodes', quantity: 4},
      ],
      runtime: [
        {name: 'Launch Models', quantity: 12},
      ],
      issues: [
        {name: 'Total', quantity: 100},
        {name: 'Source', quantity: 50},
        {name: 'Runtime', quantity: 50},
      ],
    };
  },
  methods: {
    onCustomEvent(arg1, arg2) {
      this.$emit("custom-event", arg1, arg2);
    }
  }
};
