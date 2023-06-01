// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

const DashboardHeader = {
  template: "#vue-dashboard-header",
  props: {},
  data() {
    return {
      links: [
        { text: "Source", route: "#source" },
        { text: "Runtime", route: "#runtime" },
        { text: "Issues", route: "#issues" },
      ]
    };
  },
  methods: {
    // onMenuButtonClicked(route) {
    //   this.$emit("change-route", route);
    // }
  }
};
