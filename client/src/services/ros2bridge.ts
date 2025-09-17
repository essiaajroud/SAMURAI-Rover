import ROSLIB from 'roslib';

export class ROS2Bridge {
    private ros: ROSLIB.Ros;
    private trackingSub!: ROSLIB.Topic;
    private cmdVelPub!: ROSLIB.Topic;

    constructor() {
        this.ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'  // ROS2 WebSocket bridge
        });

        this.setupTopics();
        this.setupConnections();
    }

    private setupConnections() {
        this.ros.on('connection', () => {
            console.log('Connected to ROS2 WebSocket server.');
        });
        this.ros.on('error', (error) => {
            console.error('Error connecting to ROS2 WebSocket server:', error);
        });
        this.ros.on('close', () => {
            console.log('Connection to ROS2 WebSocket server closed.');
        });
    }

    private setupTopics() {
        this.trackingSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/samurai/tracking',
            messageType: 'samurai_msgs/TrackingInfo'
        });

        this.cmdVelPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        });
    }

    public subscribeToTracking(callback: (data: any) => void) {
        this.trackingSub.subscribe(callback);
    }

    public publishCommand(linear: number, angular: number) {
        const twist = new ROSLIB.Message({
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular }
        });
        this.cmdVelPub.publish(twist);
    }
}
