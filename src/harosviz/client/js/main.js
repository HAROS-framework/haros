// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

const { createApp } = Vue;

const app = createApp({
  data() {
    return {
      title: "",
      text: "",
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
  }
});

app.component("DashboardHeader", DashboardHeader);
app.component("SetupComponent", SetupComponent);

app.mount("#app");
