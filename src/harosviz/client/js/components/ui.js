// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

const InfoPanel = {
  template: "#vue-info-panel",
  props: {
    title: {
      type: String,
      required: true
    },
    items: Array,
  }
};


const SetupComponent = {
  template: "#vue-setup-component",
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
  data() {},
  methods: {
    onCustomEvent(arg1, arg2) {
      this.$emit("custom-event", arg1, arg2);
    }
  }
};
