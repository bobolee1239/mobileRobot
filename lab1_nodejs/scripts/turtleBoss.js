#!/usr/bin/env node

const rosnodejs		= require('rosnodejs');
//rosnodejs.loadAllPackages();
const geometry_msgs	= rosnodejs.require('geometry_msgs');


rosnodejs.initNode('turtleBoss', {onTheFly: true})
.then((nodeHandle) => {
	let pub = nodeHandle.advertise('/turtle1/cmd_vel', geometry_msgs.msg.Twist);
	
	let count = 0;
	setInterval(() => {
		if(count++ % 2 === 0){
			rosnodejs.log.info('go straight');
			pub.publish({
				linear: {
					x: 3.0,
					y: 0.0,
					z: 0.0
				}, 
				angular: {
					x: 0.0,
					y: 0.0,
					z: 0.0
				}
			});
		} else {
			rosnodejs.log.info('turn around');
			pub.publish({
				linear: {
					x: 0.0,
					y: 0.0,
					z: 0.0
				}, 
				angular: {
					x: 0.0,
					y: 0.0,
					z: 1.570796327
				}
			});
		}
	}, 1000);

});
