// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

// -----------------------------------------------------------------------------
//  Constants
// -----------------------------------------------------------------------------

const { createApp } = Vue;

const ROUTES = {
  "": {
    component: "DashboardPage",
    name: "Dashboard",
    crumbs: [],
  },
  "#issues": {
    component: "IssuesPage",
    name: "Issues",
    crumbs: [""],
  },
  "#source": {
    component: "SourcePage",
    name: "Source",
    crumbs: [""],
  },
  "#nodes": {
    component: "NodesPage",
    name: "Nodes",
    crumbs: [""],
  },
  "#runtime": {
    component: "RuntimePage",
    name: "Runtime",
    crumbs: [""],
  },
};

// aliasing
ROUTES["#"] = ROUTES[""];
ROUTES["#dashboard"] = ROUTES[""];


// -----------------------------------------------------------------------------
//  Application
// -----------------------------------------------------------------------------

const app = createApp({
  data() {
    return {
      title: "",
      text: "",
      currentPage: "",
      componentWidth: 0,
      componentHeight: 0,
      crumbs: [],
      ui: {
        state: 1
      }
    };
  },

  computed: {
    isSetupState() {
      return this.ui.state === 1; // UIState.SETUP
    }
  },

  methods: {
    onSetupDone() {
    },

    onWindowHashChanged() {
      const route = ROUTES[window.location.hash] || ROUTES["#dashboard"];
      this.currentPage = route.component;
      this.text = `We are viewing the ${route.name} component.`;
      this.crumbs = [];
      for (const crumb of route.crumbs) {
        const h = `#${crumb}`;
        const r = ROUTES[h];
        this.crumbs.push({
          name: r.name,
          href: h,
        });
      }
      this.crumbs.push({ name: route.name });
    },

    onWindowResized() {
      const content = this.$refs.pageComponent;
      this.componentWidth = content.clientWidth;
      this.componentHeight = content.clientHeight;
    },

    // refreshSlides() {
    //   const slidesContainer = document.getElementById("slides-container");
    //   const slide = document.querySelector(".slide");
    //   const slideWidth = slide.clientWidth;
    //   slidesContainer.scrollLeft = this.currentTurn * slideWidth;
    // }
  },

  mounted() {
    this.title = "Vue Application";
    this.text = "Hello, world!";
    window.addEventListener("hashchange", this.onWindowHashChanged);
    this.onWindowHashChanged();
    window.addEventListener("resize", this.onWindowResized);
    this.onWindowResized();
  }
});


// -----------------------------------------------------------------------------
//  Setup
// -----------------------------------------------------------------------------

{
  for (const key of Object.keys(UI)) {
    app.component(key, UI[key]);
  }
}

app.component("NodeModelComponent", NodeModelComponent);
app.component("DashboardHeader", DashboardHeader);
app.component("DashboardPage", DashboardPage);
app.component("IssueDetails", IssueDetails);
app.component("IssuesPage", IssuesPage);
app.component("SourcePage", SourcePage);
app.component("NodesPage", NodesPage);
app.component("RuntimePage", RuntimePage);

app.mount("#app");
