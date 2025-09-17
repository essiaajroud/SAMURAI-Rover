<template>
  <!-- ...existing code... -->
</template>

<script lang="ts">
import { defineComponent } from "vue";
import { ROS2Bridge } from "@/services/ros2bridge";

export default defineComponent({
  // ...existing code...

  data() {
    return {
      ros2Bridge: new ROS2Bridge(),
      // ...existing code...
    };
  },

  mounted() {
    this.ros2Bridge.subscribeToTracking((data) => {
      this.updateTrackingData(data);
    });
  },

  methods: {
    updateTrackingData(data: any) {
      // Update tracking visualization
      // ...existing code...
    },

    sendCommand(cmd: string) {
      // Convert high-level commands to ROS2 messages
      let linear = 0,
        angular = 0;

      switch (cmd) {
        case "forward":
          linear = 0.5;
          break;
        case "turn_left":
          angular = 0.5;
          break;
        // ...existing code...
      }

      this.ros2Bridge.publishCommand(linear, angular);
    },
  },
});
</script>
