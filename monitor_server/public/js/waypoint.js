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


const add_topic = new ROSLIB.Topic({
  ros: ros,
  name: "/add_waypoint",
  messageType: "std_msgs/String"
});


const delete_topic = new ROSLIB.Topic({
  ros: ros,
  name: "/delete_waypoint",
  messageType: "std_msgs/String"
});

const save_topic = new ROSLIB.Topic({
  ros: ros,
  name: "/save_waypoints",
  messageType: "std_msgs/Empty"
});
  

let msg = new ROSLIB.Message({
  data: ''
});


function add_waypoint() {
  var data = document.getElementById('waypoint_id').value
  var is_end = document.querySelector('input[name="is_end"]:checked').value;

  data = data.toUpperCase()

  if (data == "" || data.length != 2){
    alert("Invalid Waypoint ID!!");
    return 0
  }

  temp = data.split('')

  alpha = temp[0].charCodeAt(0)
  idx = parseInt(temp[1])


  if ("A".charCodeAt(0) <= alpha && alpha <= "Z".charCodeAt(0)) {
    msg.data = data + "/" + is_end
    console.log(msg.data)

    document.getElementById('waypoint_id').value = ''
    add_topic.publish(msg)
    
    return 0
  }

  alert("Invalid Waypoint ID!!");
  return 0
}


function delete_waypoint() {
  var data = document.getElementById('waypoint_id').value
  data = data.toUpperCase()

  if (data == "" || data.length != 2){
    alert("Invalid Waypoint ID!!");
    return 0
  }

  temp = data.split('')

  alpha = temp[0].charCodeAt(0)
  idx = parseInt(temp[1])

  if ("A".charCodeAt(0) <= alpha && alpha <= "Z".charCodeAt(0)) {
    msg.data = data
  
    document.getElementById('waypoint_id').value = ''
    delete_topic.publish(msg)
    
    return 0
  }

  alert("Invalid Waypoint ID!!");
  return 0
}

function save_waypoints() {
  msg.data = ''
  save_topic.publish()
  alert("Save Waypoints!!");
}