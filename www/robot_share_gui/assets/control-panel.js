$( document ).ready(function() {
    // Create ros object to communicate over your Rosbridge connection
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    // When the Rosbridge server connects, fill the span with id "status" with "successful"
    ros.on("connection", () => {
        $("#rosbridge-status")[0].innerHTML = "successful";
    });

    // When the Rosbridge server experiences an error, fill the "status" span with the returned error
    ros.on("error", (error) => {
        $("#rosbridge-status")[0].innerHTML = `errored out (${error}), refreshing in 5s...`;
        window.setTimeout( function() {
            window.location.reload();
        }, 5000);
    });

    // When the Rosbridge server shuts down, fill the "status" span with "closed"
    ros.on("close", () => {
        $("#rosbridge-status")[0].innerHTML = "closed, refreshing in 5s...";
        window.setTimeout( function() {
            window.location.reload();
            }, 5000);
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

    // Listen to team state for each robot and position
    var robot_ids = [1,2,4,5,6,7,8,9,14,15]
    const team_listeners = {}
    const pos_listeners = {}
    for (let i = 0; i < robot_ids.length; i++) {
        id = robot_ids[i]
        team_listeners[id] = new ROSLIB.Topic({
            ros,
            name: "/turtlebot4_"+id+"/team",
            messageType: "std_msgs/Int8",
        });

        pos_listeners[id] = new ROSLIB.Topic({
            ros,
            name: "/turtlebot4_"+id+"/tb_info_topic",
            messageType: "turtlebot4_custom_msg/msg/Turtlebot4Info",
        });
    }

    pos_listeners['l0'] = new ROSLIB.Topic({
        ros,
        name: "/leader0/tb_info_topic",
        messageType: "turtlebot4_custom_msg/msg/Turtlebot4Info",
    });

    pos_listeners['l1'] = new ROSLIB.Topic({
        ros,
        name: "/leader1/tb_info_topic",
        messageType: "turtlebot4_custom_msg/msg/Turtlebot4Info",
    });

    // Listen to share requests
    const request_listener0 = new ROSLIB.Topic({
        ros,
        name: "/interface0/share_request/receive",
        messageType: "turtlebot4_custom_msg/msg/ShareRequest",
    });
    const request_listener1 = new ROSLIB.Topic({
        ros,
        name: "/interface1/share_request/receive",
        messageType: "turtlebot4_custom_msg/msg/ShareRequest",
    });

    // Listen to share accepts
    const accept_listener0 = new ROSLIB.Topic({
        ros,
        name: "/interface0/share_accept/send",
        messageType: "turtlebot4_custom_msg/msg/ShareRequest",
    });
    const accept_listener1 = new ROSLIB.Topic({
        ros,
        name: "/interface1/share_accept/send",
        messageType: "turtlebot4_custom_msg/msg/ShareRequest",
    });

    // Publish trial control
    const control_pub = new ROSLIB.Topic({
        ros,
        name: "/trial_info",
        messageType: "turtlebot4_custom_msg/msg/TrialInfo"
    });

    // ######################
    // Process robot info
    // ######################

    var robot_team = {}
    var robot_pos = {}
    var leader_pos = {}

    $.each(team_listeners,function(idx,li){ 
        li.subscribe((msg) => {
            robot_team[idx] = msg.data
        });
    });

    $.each(pos_listeners,function(idx,li){ 
        li.subscribe((msg) => {
            if (msg.role == "follower") {
                robot_pos[idx] = [
                    parseFloat(msg.x), 
                    parseFloat(msg.y)]
            } else {
                leader_pos[idx] = [
                    parseFloat(msg.x), 
                    parseFloat(msg.y)]
            }
        });
    });

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
    
    // Draw canvas

    var canvas = document.querySelector("#arena")   // Get access to HTML canvas element
    var ctx = canvas.getContext("2d")
    var canvas_w = canvas.width = 500
    var canvas_h = canvas.height = 500
    var arena_w = 680
    var arena_h = 680
    var num_teams = 2
    var num_robots = 10

    function drawBg(zone_info)
    {
        // Draw zones
        y = 0
        x = 0
        for (let i = 0; i < n_zones; i++) {
            if (i%3==0 && i!= 0) {
                x = 0
                y++
            }
            
            x0 = x*canvas_w/3
            y0 = y*canvas_h/3
            w = canvas_w/3
            h = canvas_h/3
            
            if (zone_info[i] !== undefined && zone_info[i]['status'] == 1) {
                ctx.fillStyle = "#93cfa8";
                ctx.fillRect(x0, y0, w, h);
            }
            
            ctx.lineWidth = "1";
            ctx.strokeStyle = "#dfeccc";
            ctx.rect(x0, y0, w, h);
            ctx.stroke();

            ctx.fillStyle = "#333333";
            ctx.fillText("Zone "+i, x0+10, y0+10)
            if (zone_info[i] !== undefined) {
                // ctx.fillText("Av: "+zone_info[i]['r_av'], x0+10, y0+20)
                ctx.fillText("Req: "+zone_info[i]['r_req'], x0+10, y0+20)
                ctx.fillText("Prog: "+zone_info[i]['progress'], x0+10, y0+30)
            }

            x++
        }
    }

    function translateCoord(x, y)
    {
        var x_ = (y*100+arena_w/2)*canvas_w/arena_w // real to canvas dimensions
        var y_ = canvas_h-(-x*100+arena_h/2)*canvas_h/arena_h
        return [x_, y_]
    }

    function drawPos(x, y, r, colour)
    {
        ctx.beginPath()
        ctx.fillStyle = colour
        ctx.arc(x, y, r, 0, Math.PI * 2);
        ctx.fill()
        ctx.closePath()
    }

    function clearCanvas() {
        ctx.clearRect(0, 0, canvas_w, canvas_h);
    }

    function drawFrame(zone_info, robot_pos, robot_team, leader_pos)
    {
        clearCanvas()
        drawBg(zone_info)

        for (let i = 0; i < num_robots; i++) {
            id = robot_ids[i]
            try {
                var pos = robot_pos[id]
                if (isNaN(pos[0]) || isNaN(pos[1])) {
                    continue
                }
                
                var c = translateCoord(pos[0], pos[1])
                
                if (robot_team[id] == 0) {
                    colour = 'blue';
                } else if (robot_team[id] == 1) {
                    colour = 'red'
                }
                
                drawPos(c[0],c[1],5,colour)
            } catch {
                
            }
        }

        for (let i = 0; i < num_teams; i++) {
            try {
                var pos = leader_pos["l"+i]
                if (isNaN(pos[0]) || isNaN(pos[1])) {
                    continue
                }

                var c = translateCoord(pos[0], pos[1])
                
                if (i == 0) {
                    colour = 'blue';
                } else if (i == 1) {
                    colour = 'red'
                }
                
                drawPos(c[0],c[1],7,colour)
            } catch {
                //
            }
        }
    }

    // ########################################
    // Populate the request info panel
    // ########################################

    function datetime(unix_timestamp) {
        // Create a new JavaScript Date object based on the timestamp
        // multiplied by 1000 so that the argument is in milliseconds, not seconds
        var date = new Date(unix_timestamp * 1000);

        // Hours part from the timestamp
        var hours = date.getHours();

        // Minutes part from the timestamp
        var minutes = "0" + date.getMinutes();

        // Seconds part from the timestamp
        var seconds = "0" + date.getSeconds();

        // Will display time in 10:30:23 format
        var formattedTime = hours + ':' + minutes.substring(-2) + ':' + seconds.substring(-2);
        return formattedTime
    }

    // Request sent from 0
    request_listener0.subscribe((msg) => {
            $("req_s0")[0].innerHTML = datetime(msg.timestamp)
        });

    request_listener1.subscribe((msg) => {
            $("req_s1")[0].innerHTML = datetime(msg.timestamp)
        });

    accept_listener0.subscribe((msg) => {
            if (msg.data == 3) {
                // Accept immediately
                msg_str = datetime(msg.timestamp)
            }

            if (msg.data == 4) {
                // Accept with delay
                msg_str = datetime(msg.timestamp) + " with delay"
            }
            
            $("acc0")[0].innerHTML = msg_str
        });
        
    accept_listener1.subscribe((msg) => {
            if (msg.data == 3) {
                // Accept immediately
                msg_str = datetime(msg.timestamp)
            }

            if (msg.data == 4) {
                // Accept with delay
                msg_str = datetime(msg.timestamp) + " with delay"
            }
            
            $("acc1")[0].innerHTML = msg_str
        });

    // ########################################
    // Manage control panel buttons
    // ########################################
    
    var control_state = -1
    $("#control-panel button").click(function() {
        control_state = parseInt(this.value)
        if (control_state == 4 || control_state == 5 || control_state == 3) {
            $('#r-state')[0].innerHTML = this.innerHTML
        } else {
            $('#t-state')[0].innerHTML = this.innerHTML
        }
    });
    
    // Ping pub
    interval = window.setInterval(function(){
        var msg = new ROSLIB.Message({
            state: control_state
        });
        control_pub.publish(msg);

        drawFrame(zone_info, robot_pos, robot_team, leader_pos)  
    }, 100);

});

