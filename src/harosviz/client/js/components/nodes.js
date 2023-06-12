// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

const NodesPage = {
  template: "#vue-nodes-page",
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
      nodes: {
        "package_1/node_1": null,
        "package_1/node_2": null,
        "package_2/node_3": null,
        "package_3/node_4": null,
      },
      nodeList: [],
      selectedNode: null,
    };
  },
  computed: {
    packageList() {
      const packages = {};
      for (const key of Object.keys(this.nodes)) {
        const pkg = key.split("/")[0];
        packages[pkg] = true;
      }
      const sortedPackages = Object.keys(packages).sort();
      const displayPackages = [];
      for (const name of sortedPackages) {
        displayPackages.push({
          name: name,
          value: name,
        });
      }
      return displayPackages;
    },

    isNodeSelected() {
      return this.package != "" && this.node != "";
    }
  },

  methods: {
    getNodeList(pkg) {
      const nodes = [];
      for (const key of Object.keys(this.nodes)) {
        const parts = key.split("/");
        if (pkg === parts[0]) {
          nodes.push({
            name: parts[1],
            value: parts[1],
          });
        }
      }
      return nodes;
    },

    onPackageSelected(pkg) {
      this.nodeList = this.getNodeList(pkg);
      this.$refs.nodeMenu.clearSelection();
      this.selectedNode = null;
    },

    onNodeSelected(node) {
      if (!!node) {
        const pkg = this.$refs.packageMenu.getSelectedValue();
        this.selectedNode = `${pkg}/${node}`;
      } else {
        this.selectedNode = null;
      }
    }
  },

  mounted() {
    d3.select(this.$refs.modelContainer).append(window.exampleNodeModel);
  }
};
