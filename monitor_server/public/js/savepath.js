const ros = new ROSLIB.Ros();


ros.connect(`ws://${window.location.host.split(":")[0]}:9090`);


ros.on('error', (error) => {
  console.log(error);
});


ros.on('connection', (error) => {
  console.log('Connection: ok!');
});


ros.on('close', (error) => {
  console.log('Connection closed.');
});


// Subscriber => /waypoints, visualization_msgs/MarkerArray
// Subscriber => saving_check, String
// Publisher => PathPoint, PathRequest
// Publisher => saving_ans, UInt8

const requestPub = new ROSLIB.Topic({
  ros: ros,
  name: "/PathPoint",
  messageType: "path_plan/PathRequest"
});

const ansPub = new ROSLIB.Topic({
    ros: ros,
    name: "/saving_ans",
    messageType: "std_msgs/UInt8"
});

const saveSub = new ROSLIB.Topic({
    ros: ros,
    name: "/saving_check",
    messageType: "std_msgs/String"
});

const markerSub = new ROSLIB.Topic({
    ros: ros,
    name: "/waypoints",
    messageType: "visualization_msgs/MarkerArray"
});

let req = new ROSLIB.Message({
    start: "",
    end: "",
    path_id: ""
  });

let answer = new ROSLIB.Message({
    data: 0
});

function savePath() {
    var start = document.getElementById('start').value
    var end = document.getElementById('end').value

    req.start = start
    req.end = end
    req.path_id = start + end

    document.getElementById('start').value = ""
    document.getElementById('end').value = ""

    requestPub.publish(req)
}


window.onload = () => {
    saveSub.subscribe((res) => {
        let msg = res.data
        let selection = confirm(msg)

        answer.data = selection == true ? 1 : 0

        ansPub.publish(answer)

    })
}