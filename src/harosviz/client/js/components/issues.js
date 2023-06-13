// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

const IssueDetails = {
  template: "#vue-issue-details",
  props: {
    message: {
      type: String,
      required: true
    },
    details: String,
    location: Object
  },

  computed: {
    issueNumber() {
      return this.$.vnode.key;
    },

    sourceLocationString() {
      if (!this.location || !this.location.package) {
        return "<location unknown>"
      }
      let text = `in package ${this.location.package}`;
      if (!this.location.file) {
        return text;
      }
      text = `in ${this.location.package}/${this.location.file}`;
      if (!this.location.line) {
        return text;
      }
      text = text + `, line ${this.location.line}`;
      if (!this.location.column) {
        return text;
      }
      return text + `, column ${this.location.column}`;
    }
  }
};


const IssuesPage = {
  template: "#vue-issues-page",
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
      issues: [
        {
          message: "Memory Leak",
          location: null,
          details: ""
        },
        {
          message: "Index Out Of Bounds",
          location: null,
          details: "Index 9 used on an array of length 8."
        },
        {
          message: "Type Error",
          location: {
            package: "package_1",
            file: "src/main.py",
            line: 15,
            column: 61
          },
          details: ""
        },
        {
          message: "Invocation Error",
          location: {
            package: "package_1",
            file: "src/main.py",
            line: 15,
            column: 61
          },
          details: "This function expects 3 arguments but only 2 were supplied."
        },
      ]
    };
  },

  methods: {
    convertIndexNumber(i) {
      const n = this.issues.length.toString().length;
      return (i + 1).toString().padStart(n, "0");
    }
  }
};
