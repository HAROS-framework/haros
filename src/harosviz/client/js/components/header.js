// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

const DashboardHeader = {
  template: "#vue-dashboard-header",
  props: {},
  data() {},
  methods: {
    onCustomEvent(arg1, arg2) {
      this.$emit("custom-event", arg1, arg2);
    }
  }
};
