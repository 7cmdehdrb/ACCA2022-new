const ros = new ROSLIB.Ros();


ros.connect('ws://localhost:9090');


ros.on('error', (error) => {

  console.log(error);

});


ros.on('connection', (error) => {

  console.log('Connection: ok!');

});


ros.on('close', (error) => {

  console.log('Connection closed.');

});


const topic = new ROSLIB.Topic({

  ros: ros,

  name: '/topic',

  messageType: 'std_msgs/String'

});


let msg = new ROSLIB.Message({

    data: ''

});


function publish() {    

    msg.data = document.getElementById('msg').value

    document.getElementById('msg').value = ''

    topic.publish(msg)

}


window.onload = () => {

    topic.subscribe((res) => {

        document.getElementById('subscribe').innerHTML = res.data

    })

}