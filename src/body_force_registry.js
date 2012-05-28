var Mathematics = require("goom-math");

/**
	Creates a BodyForceRegistry.
	@class This class is used to register the force generators that affect ay given particles
	in the physics engine.
	@exports BodyForceRegistry as Physics.BodyForceRegistry.
	@property {Array} registrations An array holding the body and force generator pairs,
	will hold the pair as an object with a body key to the rigid body and a
	forceGenerator key to the ForceGenerator.
*/
function BodyForceRegistry() {
	this.registrations = [];
}

/**
	Adds a new RigidBody and ForceGenerator pair to the registry.
	@param {Physics.RigidBody} body The rigid body.
	@param {Physics.ForceGenerator} force_generator The force generator affecting the given
	rigid body.
*/
BodyForceRegistry.prototype.add = function(body, force_generator) {
	this.registrations.push({"body": body, "forceGenerator": force_generator});
};

/**
	Removes a RigidBody and ForceGenerator pair from the registry.
	@param {Physics.RigidBody} body The rigid body.
	@param {Physics.ForceGenerator} force_generator The force generator affecting the given
	rigid body.
*/
BodyForceRegistry.prototype.remove = function(body, force_generator) {
	var i, r, len;
	for (i = 0, len = this.registrations.length; i < len; i++) {
		r = this.registrations[i];
		if (r.body === body && r.forceGenerator === force_generator) {
			this.registrations.splice(i, 1);
			return;
		}
	}
};

/**
	Clears all the registrations in the registry.
*/
BodyForceRegistry.prototype.clear = function() {
	var i, r, len;
	for (i = 0, len = this.registrations.length; i < len; i++) {
		this.registrations.pop();
	}
};

/**
	Calls all the force generators to update the corresponding body's forces.
	@param {Number} Duration of the applied force.
*/
BodyForceRegistry.prototype.updateForces = function(duration) {
	var r, i, len, ref;
	ref = this.registrations;
	
	for (i = 0, len = ref.length; i < len; i++) {
		r = ref[i];
		r.forceGenerator.updateForce(r.body, duration);
	}
};

module.exports = BodyForceRegistry;