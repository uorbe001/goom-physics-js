var Mathematics = require("goom-math-js"), ForceGenerator = require("./force_generator");

var __hasProp = Object.prototype.hasOwnProperty, __extends = function(child, parent) {
	for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; }
	function ctor() { this.constructor = child; }
	ctor.prototype = parent.prototype;
	child.prototype = new ctor();
	child.__super__ = parent.prototype;
	return child;
};

/**
	Creates a gravity force generator.
	@class Used to apply gravity to the bodies in the world.
	@exports Gravity as Physics.Gravity
	@param {Math.Vector3D} gravity The acceleration due to gravity.
	@property {Math.Vector3D} gravity The acceleration due to gravity.
*/
var Gravity = (function() {
	__extends(Gravity, ForceGenerator);

	function Gravity(gravity) {
		this.gravity = gravity;
	}

	/**
		This is an internal varible used to avoid the creation of new objects at runtime.
		@inner
	*/
	__vector = new Mathematics.Vector3D();

	/**
		Calculates and updates the force applied to the given rigid body.
		@param {Physics.RigidBody} body The rigid body where the given force will be applied.
		@param {Number} duration  Duration of the applied force.
	*/
	Gravity.prototype.updateForce = function(body, duration) {
		if (!body.hasFiniteMass()) return;
		body.applyForce(this.gravity.scale(body.getMass(), __vector));
	};

	return Gravity;
})();

module.exports = Gravity;