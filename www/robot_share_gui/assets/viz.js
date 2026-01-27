$( document ).ready(function() {
    // Create ros object to communicate over your Rosbridge connection
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    // When the Rosbridge server connects, fill the span with id "status" with "successful"
    ros.on("connection", () => {
        console.log('successful')
        // $("#rosbridge-status")[0].innerHTML = "successful";
    });

    // When the Rosbridge server experiences an error, fill the "status" span with the returned error
    ros.on("error", (error) => {
        console.log('errored')
        // $("#rosbridge-status")[0].innerHTML = `errored out (${error})`;
    });

    // When the Rosbridge server shuts down, fill the "status" span with "closed"
    ros.on("close", () => {
        console.log('closed')
        // $("#rosbridge-status")[0].innerHTML = "closed";
    });

    // Create a listener for task info
    const n_zones = 9
    const zone_listeners = {}
    for (let i = 0; i < n_zones; i++) {
        zone_listeners[i] = new ROSLIB.Topic({
            ros,
            name: "/zone"+i+"/task_info",
            messageType: "turtlebot4_custom_msg/msg/TaskInfo",
        });
    }

    const share_points_listener = {}
    const task_points_listener = {}

    for (let i = 0; i < 2; i++) {
        share_points_listener[i] = new ROSLIB.Topic({
            ros,
            name: "/share_points/team"+i,
            messageType: "std_msgs/Int16",
        });
        task_points_listener[i] = new ROSLIB.Topic({
            ros,
            name: "/task_points/team"+i,
            messageType: "std_msgs/Int16",
        });
    }

    // ######################
    // Process zone info
    // ######################

    var zone_info = {}
    for (let i = 0; i < n_zones; i++) {
        zone_listeners[i].subscribe((msg) => {
            zone_info[i] = {
                'status': msg.status,
                'r_av': msg.robots_available,
                'r_req': msg.robots_required,
                'progress': msg.progress
            }
        });
    }

    // ========================
    // Listen to points
    // ========================

    var share_points = [0,0]
    var task_points = [0,0]

    for (let i = 0; i < 2; i++) {
        share_points_listener[i].subscribe((msg) => {
            share_points[i] = msg.data
        })
        task_points_listener[i].subscribe((msg) => {
            task_points[i] = msg.data
        })
    }

    interval = window.setInterval(function(){
        // Check which zones are active and display
        for (let i = 0; i < n_zones; i++) {
            if (zone_info[i] == undefined) {
                continue
            }
            
            stat = zone_info[i]['status']
            if (stat == 1) {
                $('#b'+i).addClass('active')
            } else {
                $('#b'+i).removeClass('active')
            }
        }

        total_points = share_points[0] + share_points[1] + task_points[0] + task_points[1]
        $("#score p")[0].innerHTML = String(total_points)
    }, 100);

});