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


const loggerPub = new ROSLIB.Topic({
  ros: ros,
  name: "/gps_logger/logging",
  messageType: "std_msgs/Empty"
});

const msg = new ROSLIB.Message()


function publish() {
  loggerPub.publish(msg)
}

