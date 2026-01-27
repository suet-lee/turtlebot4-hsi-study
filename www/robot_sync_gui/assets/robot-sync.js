$( document ).ready(function() {
    // Create ros object to communicate over your Rosbridge connection
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    // When the Rosbridge server connects, fill the span with id "status" with "successful"
    ros.on("connection", () => {
        $("#rosbridge-status")[0].innerHTML = "successful";
        $("#sliders").show()
    });

    // When the Rosbridge server experiences an error, fill the "status" span with the returned error
    ros.on("error", (error) => {
        $("#rosbridge-status")[0].innerHTML = `errored out (${error})`;
        $("#sliders").hide()
    });

    // When the Rosbridge server shuts down, fill the "status" span with "closed"
    ros.on("close", () => {
        $("#rosbridge-status")[0].innerHTML = "closed";
        $("#sliders").hide()
    });

    // TODO default v and radius on init
    // Topic to publish to (interface -> ros)
    // Topic: /team_info/team{id}

    // Get interface id
    const id = $("#interface-id").val();
    if (id != "0" && id != "1") {
        throw new Error("Invalid id");
    }

    // // Create a publisher to send team info
    const publisher = new ROSLIB.Topic({
        ros,
        name: "/team_info/team"+id,
        messageType: "turtlebot4_custom_msg/msg/TeamInfo",
    });

    // Create a listener for initialising team info
    const info_listener = new ROSLIB.Topic({
        ros,
        name: "/team_info/team"+id+"/out",
        messageType: "turtlebot4_custom_msg/msg/TeamInfo",
    });

    // Create a listener for success status
    const success_listener = new ROSLIB.Topic({
        ros,
        name: "/team_info/sync_success",
        messageType: "std_msgs/Int8",
    });

    const min_v = 0.1
    const max_v = 1.0
    const min_r = 0.5
    const max_r = 1.0
    var current_v = 0.5
    var current_r = 1.0

    $("#radius,#velocity").on('change', function(val) {
        r = $("#radius").val();
        v = $("#velocity").val();

        r_ = ((max_r-min_r)*r/100 + min_r)
        r_ = Math.round(r_*10)/10
        v_ = ((max_v-min_v)*v/100 + min_v)
        v_ = Math.round(v_*10)/10

        $("#radius-value")[0].innerHTML = r_
        $("#velocity-value")[0].innerHTML = v_

        current_v = v_
        current_r = r_
    });

    info_listener.subscribe((msg) => {
        v = msg.v
        $("#velocity-value")[0].innerHTML = v
        slider_v = 100*(v - min_r)/(max_r-min_r)
        $("#velocity")[0].value = slider_v

        radius = msg.radius
        $("#radius-value")[0].innerHTML = radius
        slider_r = 100*(radius - min_r)/(max_r-min_r)
        $("#radius")[0].value = slider_r
    });

    success_listener.subscribe((msg) => {
        console.log("sync success")
        if (msg.data == 1) {
            // $("#notification p.msg").show()
            $("#notification p.msg").fadeIn()
        }
        else {
            $("#notification p.msg").fadeOut('fast')
        }
    });

    interval = window.setInterval(function(){
        var msg = new ROSLIB.Message({
            radius: current_r, v: current_v
        });

        publisher.publish(msg);        
    }, 100);

});

