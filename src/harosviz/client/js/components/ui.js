// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

// -----------------------------------------------------------------------------
//  Exports
// -----------------------------------------------------------------------------

const UI = {};

// -----------------------------------------------------------------------------
//  General UI Components
// -----------------------------------------------------------------------------

UI.InfoPanel = {
  template: "#vue-info-panel",
  props: {
    title: {
      type: String,
      required: true
    },
    items: Array,
  }
};


UI.DropdownMenu = {
  template: "#vue-dropdown-menu",
  props: {
    label: {
      type: String,
      required: true
    },
    options: {
      type: Array,
      default(rawProps) { return []; },
    },
  },
  methods: {
    onOptionSelected(e) {
      this.$emit("optionSelected", e.target.value);
    },

    getSelectedValue() {
      return this.$refs.dropdown.value;
    },

    clearSelection() {
      this.$refs.dropdown.value = "";
    }
  }
};


// -----------------------------------------------------------------------------
//  Tree View
// -----------------------------------------------------------------------------

UI.TreeItem = {
  name: 'TreeItem', // necessary for self-reference
  template: "#vue-tree-item",
  props: {
    model: Object
  },
  data() {
    return {
      isOpen: false
    }
  },
  computed: {
    isFolder() {
      return this.model.children && this.model.children.length
    }
  },
  methods: {
    toggle() {
      if (this.isFolder) {
        this.isOpen = !this.isOpen
      }
    },
    changeType() {
      if (!this.isFolder) {
        this.model.children = []
        this.addChild()
        this.isOpen = true
      }
    },
    addChild() {
      this.model.children.push({
        name: 'new stuff'
      })
    }
  }
};


UI.TreeView = {
  template: "#vue-tree-view",
  components: {
    TreeItem: UI.TreeItem
  },
  props: {
    tree: {
      type: Object,
      required: true,
    }
  }
};


// -----------------------------------------------------------------------------
//  Template
// -----------------------------------------------------------------------------

UI.SetupComponent = {
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
