$( document ).ready(function() {
    // Create ros object to communicate over your Rosbridge connection
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    // When the Rosbridge server connects, fill the span with id "status" with "successful"
    ros.on("connection", () => {
        $("#rosbridge-status")[0].innerHTML = "successful";
    });

    // When the Rosbridge server experiences an error, fill the "status" span with the returned error
    ros.on("error", (error) => {
        $("#rosbridge-status")[0].innerHTML = `errored out (${error})`;
    });

    // When the Rosbridge server shuts down, fill the "status" span with "closed"
    ros.on("close", () => {
        $("#rosbridge-status")[0].innerHTML = "closed";
    });

    // Topic to subscribe to (ros -> interface)
    // Topic: /interface{id}/share_request/send
    // Topic: /interface{id}/task_info
    // ----
    // Topic to publish to (interface -> ros)
    // Topic: /interface{id}/share_request/receive

    // Get interface id
    const id = $("#interface-id").val();
    if (id != "0" && id != "1") {
        throw new Error("Invalid id");
    }

    // Create a listener for incoming share requests
    const request_listener = new ROSLIB.Topic({
        ros,
        name: "/interface"+id+"/share_request/to_interface",
        messageType: "turtlebot4_custom_msg/msg/ShareRequest",
    });

    // Create a listener for task info
    const task_listener = new ROSLIB.Topic({
        ros,
        name: "/interface"+id+"/task_info",
        messageType: "turtlebot4_custom_msg/msg/TaskInfo",
    });

    const interface_state_listener = new ROSLIB.Topic({
        ros,
        name: "/interface"+id+"/state",
        messageType: "std_msgs/Int8",
    });

    // Create a publisher to send share requests
    const request_publisher = new ROSLIB.Topic({
        ros,
        name: "/interface"+id+"/share_request/to_node",
        messageType: "turtlebot4_custom_msg/msg/ShareRequest",
    });

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

    // ========================
    // Handle incoming requests and outgoing responses
    // ========================

    var request_received = false;
    var request_id = -1
    var transfer_now = false;
    var transfer_delay = false;
    var teammate_cancel = false;
    var robots_available = 0

    // On request received, display request and activate response buttons:
    request_listener.subscribe((msg) => {
        if (transfer_now || transfer_delay) { // User already responded
            return;
        }

        if (msg.data == 2) {
            // Request cancelled
            console.log("Request cancelled ", msg);
            if (!teammate_cancel) {
                $("p.notification").remove()
                $("#message-box p").prepend("<p class='notification'>Teammate has cancelled request</p>");
            }
            
            request_received = false;
            transfer_now = false;
            transfer_delay = false;
            teammate_cancel = true;
            
            resetShareBtns();
            
            // setTimeout(function() {
            //     $("p.notification").remove()
            // }, 5000);
            return;
        }
        
        // Otherwise the message is a new request
        
        // Check if it's a new request
        if (request_id == msg.request_id) {
            return;
        }

        console.log("Request received");
        $("#message-box p")[0].innerHTML = "Your teammate is requesting more robots";
        request_received = true;
        teammate_cancel = false;
        request_id = msg.request_id

        $("#share-box .btn").removeClass("disabled");
        
        // setTimeout(function() {
        //     console.log("Request timed out");
        //     $("#message-box p").prepend("<p>Request timed out</p>")
        // }, 300000); // 1000 = 1s
        
    });

    function resetShareBtns() {
        console.log('resetting btns..')
        $("#message-box p")[0].innerHTML = "No new requests received";
        $("#share-box .btn").addClass("disabled");
        request_received = false;
    }

    $("#btn-transfer-now").click(function(){
        console.log("Accepted request: transfer now")
        transfer_now = true;
        resetShareBtns()
        $("#message-box p").prepend("<p class='notification'>Request accepted: transfer now</p>")
        
        setTimeout(function() {
            $("p.notification").remove()
        }, 300000); // 1000 = 1s
    });

    $("#btn-transfer-delay").click(function(){
        console.log("Accepted request: transfer on task completion")
        transfer_delay = true;
        $("#share-box .btn").addClass("disabled")
        $("#message-box p").prepend("<p class='notification'>Request accepted: transfer on task completion</p>");
        resetShareBtns()

        setTimeout(function() {
            $("p.notification").remove()
        }, 300000); // 1000 = 1s
    });

    // accept_conf.subscribe((msg) => {
    //     console.log("accepted?")
    //     // Request acceptance has gone through
    //     // The share request should also have been published
    //     // Therefore can reset buttons
    //     resetShareBtns();
    // });

    // ========================
    // Process task information
    // ========================

    var task_id = -1;
    var task_status = 0;
    var robots_available = 0;
    var robots_required = 0;
    var task_progress = 0;

    // Incoming info on task
    task_listener.subscribe((msg) => {
        new_task_id = parseInt(msg.id);
        if (task_id != new_task_id && transfer_delay) {
            // Trigger transfer since task has changed
            transfer_now = true;
            transfer_delay = false;      
            $("p.notification").remove()
            $("#message-box p").prepend("<p class='notification'>Task changed: transfer initiated</p>");
            
            setTimeout(function() {
                $("p.notification").remove()
            }, 5000);
        }
        
        task_id = new_task_id;
        task_status = msg.status;
        robots_available = msg.robots_available;
        robots_required = msg.robots_required;
        task_progress = msg.progress;

        if (robots_available == 6) {
            $("#btn-send-rqt,#btn-cancel-rqt").addClass("disabled");
        } else if (!request_cancelled) {
            if (!request_sent) {
                $("#btn-send-rqt").removeClass("disabled");
            }
            $("#btn-cancel-rqt").removeClass("disabled");
        }

        $("#task-robots")[0].innerHTML = robots_available+"/"+robots_required+" robots in zone";
        
        if (task_status == "1") {
            if (parseInt(robots_available) >= parseInt(robots_required)) {
                task_msg = "Task in progress / Team requirements met";
            } else {
                task_msg = "Active zone / Team requirements not met";
            }
        } else {
            task_msg = "Not in active task zone";
        }
        $("#task-msg")[0].innerHTML = task_msg;

        $("#task-progress .complete").css('width',task_progress+"%")
        
        // Check task completion
        if (task_progress >= 99) {
            if (transfer_delay) {
                transfer_now = true;
                transfer_delay = false;

                $("p.notification").remove()
                $("#message-box p").prepend("<p class='notification'>Task complete: transfer initiated</p>");

            }
            else {
                $("#message-box p").prepend("<p class='notification'>Task complete</p>");
            }                
            setTimeout(function() {
                $("p.notification").remove()
            }, 5000);
        }
    });

    // ========================
    // Send requests
    // ========================

    var request_sent = false;
    var request_cancelled = false;

    $("#btn-send-rqt").click(function() {

        $(this).addClass("disabled");
        request_sent = true;
        // $("#btn-send-rqt p")[0].innerHTML = "Request sent";
        
        // setTimeout(function() {
        //     $(this).removeClass("disabled");
            // $("#btn-send-rqt p")[0].innerHTML = "Send request <i class='fa-solid fa-bell-concierge'></i>";
            // request_sent = false;
        // }, 10000);
    });

    $("#btn-cancel-rqt").click(function() {
        console.log("request cancelled")

        $("#btn-send-rqt,#btn-cancel-rqt").addClass("disabled")
        request_sent = false;
        request_cancelled = true;

        setTimeout(function() {
            $("#btn-send-rqt,#btn-cancel-rqt").removeClass("disabled");
            request_cancelled = false;
            console.log("request cancelled^2")
        }, 5000);
    });

    // Listens to if send request is resolved
    interface_state_listener.subscribe((msg) => {
        if (msg.data == 5 && robots_available < 6) {
            // Teammate has accepted a request
            // Stop sending request
            request_sent = false;
            request_cancelled = false;
            $("#btn-send-rqt").removeClass("disabled");
        }

        if (msg.data == 6 && transfer_now) {
            // Accept has been processed
            // Stop sending accept
            transfer_now = false;
            $("#share-box .btn").addClass("disabled")
        }
    })

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
    
    // ========================
    // Continuous pub
    // ========================

    interval = window.setInterval(function(){
        if (request_sent) {
            console.log("send")
            sendRequest(request_publisher, 1)
        }
        else if (request_cancelled) {
            console.log("cancel")
            sendRequest(request_publisher, 2)
        }
        else {
            // Send idle state to node
            // Used to stop cancel signal to other interface
            console.log("reset send state")
            sendRequest(request_publisher, 5) // SIGNAL_SEND_RESOLVED
        }
        
        if (transfer_now) {
            sendRequest(request_publisher, 3)
        }

        if (transfer_delay) {
            sendRequest(request_publisher, 4)
        }

        total_points = share_points[0] + share_points[1] + task_points[0] + task_points[1]
        $("#points")[0].innerHTML = String(total_points)
        
    }, 100);

    function sendRequest(publisher, response) {
        var msg = new ROSLIB.Message({
            sender_id: parseInt(id),
            data: response
        });

        publisher.publish(msg);
    }

});

