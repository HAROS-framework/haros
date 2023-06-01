// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

const { createApp } = Vue;

const ROUTES = {
  "": "DashboardPage",
  "#dashboard": "DashboardPage",
  "#issues": "IssuesPage",
  "#source": "SourcePage",
  "#runtime": "RuntimePage",
};

const app = createApp({
  data() {
    return {
      title: "",
      text: "",
      currentPage: "SetupComponent",
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

    onRouteChanged(route) {
      alert(`Changed route to: ${route}`);
    },

    onWindowHashChanged() {
      const page = ROUTES[window.location.hash];
      if (page !== undefined) {
        this.currentPage = page;
        this.text = `We are viewing the ${page} component.`;
      }
    }

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
  }
});

app.component("DashboardHeader", DashboardHeader);
app.component("DashboardPage", DashboardPage);
app.component("IssuesPage", IssuesPage);
app.component("SourcePage", SourcePage);
app.component("RuntimePage", RuntimePage);
app.component("SetupComponent", SetupComponent);

app.mount("#app");
