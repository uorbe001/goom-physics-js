var RigidBody = require("./rigid_body");

/**
	Creates a force generator.
	@class Used to add forces to one or more bodies. This class is just a interface, should
	be expanded and implemented by actual generators.
	@exports ForceGenerador as Physics.ForceGenerator
*/
function ForceGenerator() {}

/**
	Calculates and updates the force applied to the given rigid body.
	@param {Physics.RigidBody} body The rigid body where the given force will be applied.
	@param {Number} duration  Duration of the applied force.
*/
ForceGenerator.prototype.updateForce = function(body, duration) {};
module.exports = ForceGenerator;