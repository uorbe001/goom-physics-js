var Mathematics = require("goom-math");

/**
	Creates a ContactResolver.
	@class This class is used to resolve detected contacts.
	@exports ContactResolver as Physics.ContactResolver
	@property {Number} MAX_VELOCITY_ITERATIONS The maximum ammount of iterations to perform when resolving velocity.
	@property {Number} MAX_POSITION_ITERATIONS The maximum ammount of iterations to perform when resolving position.
	@property {Number} VELOCITY_EPSILON In order to avoid instability, velocity values smaller than this are considered 0. Should be set on constructor and remain constant.
	@property {Number} POSITION_EPSILON In order to avoid instability, position values smaller than this are considered 0.. Should be set on constructor and remain constant.
	@param {Number} [VELOCITY_EPSILON=0.01] In order to avoid instability, velocity values smaller than this are considered 0. Should be set on constructor and remain constant.
	@param {Number} [POSITION_EPSILON=0.01] In order to avoid instability, position values smaller than this are considered 0. Should be set on constructor and remain constant.
*/
function ContactResolver(VELOCITY_EPSILON, POSITION_EPSILON) {
	this.VELOCITY_EPSILON = VELOCITY_EPSILON !== null && VELOCITY_EPSILON !== undefined? VELOCITY_EPSILON : 0.01;
	this.POSITION_EPSILON = POSITION_EPSILON !== null && POSITION_EPSILON !== undefined? POSITION_EPSILON : 0.01;
	this.MAX_VELOCITY_ITERATIONS = 0;
	this.MAX_POSITION_ITERATIONS = 0;
	this.__linearChange = new Array(2);
	this.__angularChange = new Array(2);
	this.__linearChange[0] = new Mathematics.Vector3D();
	this.__linearChange[1] = new Mathematics.Vector3D();
	this.__angularChange[0] = new Mathematics.Vector3D();
	this.__angularChange[1] = new Mathematics.Vector3D();
	this.__helperVector = new Mathematics.Vector3D();
	this.__helperVector2 = new Mathematics.Vector3D();
}

/**
	Resolves the velocity and position for a set of contacts. Contacts that cannot interact with each other should be passed in
	separate calls, since the algorithm takes much longer for lots of contacts than it does for the same number of contacts in
	small sets.
	@param {Array} contacts Array holding the contacts to resolve.
	@param {Number} duration The duration of the previous integration step.
*/
ContactResolver.prototype.resolve = function(contacts, duration) {
	if (contacts.length === 0) return;
	//Reset the max iterations for better resolution
	this.MAX_VELOCITY_ITERATIONS = this.MAX_POSITION_ITERATIONS = contacts.length * 2;
	
	var i, len, j, len2;
	//Prepare the contacts for processing, calculating the internal data
	for (i = 0, len = contacts.length; i < len; i++) {
		contact = contacts[i];
		contact.calculateInternalData(duration);
	}

	var iterations_used = 0, max, index, chosen_contact, contact, delta_position;
	//Resolve the interpenetration problems with the contacts
	while (iterations_used < this.MAX_POSITION_ITERATIONS) {
		//Find biggest penetration
		max = this.POSITION_EPSILON;
		index = contacts.length;

		for (i = 0, len = contacts.length; i < len; i++) {
			if (contacts[i].penetration > max) {
				max = contacts[i].penetration;
				index = i;
			}
		}

		//No positions to adjust, they are all smaller than the POSITION_EPSILON
		if (index === contacts.length) break;
		
		//Match the awake state of the contact.
		chosen_contact = contacts[index];
		chosen_contact.matchAwakeState();
		//Resolve penetration
		chosen_contact.resolvePosition(this.__linearChange, this.__angularChange, max);

		//The previous actions may have changed the penetration of other bodies, so we need to update the contacts
		for (j = 0, len2 = contacts.length; j < len2; j++) {
			contact = contacts[j];

			if ((contact.bodies[0] === chosen_contact.bodies[0]) && (!contact.bodies[0].isStatic)) {
				delta_position = this.__linearChange[0].add(this.__angularChange[0].crossProduct(contact.relativeContactPositions[0], this.__helperVector), this.__helperVector);
				contact.penetration += delta_position.dotProduct(contact.normal.scale(-1, this.__helperVector2));
			}

			if ((contact.bodies[0] === chosen_contact.bodies[1]) && (!contact.bodies[0].isStatic)) {
				delta_position = this.__linearChange[1].add(this.__angularChange[1].crossProduct(contact.relativeContactPositions[0], this.__helperVector), this.__helperVector);
				contact.penetration += delta_position.dotProduct(contact.normal.scale(-1, this.__helperVector2));
			}

			if ((contact.bodies[1] !== null && contact.bodies[1] !== undefined) && (!contact.bodies[1].isStatic)) {
				if (contact.bodies[1] === chosen_contact.bodies[0]) {
					delta_position = this.__linearChange[0].add(this.__angularChange[0].crossProduct(contact.relativeContactPositions[1], this.__helperVector), this.__helperVector);
					contact.penetration += delta_position.dotProduct(contact.normal);
				}

				if ((contact.bodies[1] === chosen_contact.bodies[1]) && (!contact.bodies[1].isStatic)) {
					delta_position = this.__linearChange[1].add(this.__angularChange[1].crossProduct(contact.relativeContactPositions[1], this.__helperVector), this.__helperVector);
					contact.penetration += delta_position.dotProduct(contact.normal);
				}
			}
		}

		iterations_used += 1;
	}

	//Resolve the velocity
	iterations_used = 0;
	var delta_velocity;

	while (iterations_used < this.MAX_VELOCITY_ITERATIONS) {
		//Find contact with minimum probability of velocity change
		max = this.VELOCITY_EPSILON;
		index = contacts.length;

		for (i = 0, len = contacts.length; i < len; i++) {
			if (contacts[i].desiredDeltaVelocity > max) {
				max = contacts[i].desiredDeltaVelocity;
				index = i;
			}
		}

		//No velocities to adjust, they are all smaller than VELOCITY_EPSILON
		if (index === contacts.length) break;
		//Match the awake state of the contact.
		chosen_contact = contacts[index];
		chosen_contact.matchAwakeState();
		//Resolve velocity
		chosen_contact.resolveVelocity(this.__linearChange, this.__angularChange);
		//Recompute some of the relative closing velocities
		for (j = 0, len2 = contacts.length; j < len2; j++) {
			contact = contacts[j];

			if (contact.bodies[0] === chosen_contact.bodies[0]) {
				delta_velocity = this.__angularChange[0].crossProduct(contact.relativeContactPositions[0], this.__helperVector);
				delta_velocity.add(this.__linearChange[0]);
				contact.velocity.add(contact.toWorld.transformTransposeVector(delta_velocity));
			}

			if (contact.bodies[0] === chosen_contact.bodies[1]) {
				delta_velocity = this.__angularChange[1].crossProduct(contact.relativeContactPositions[0], this.__helperVector);
				delta_velocity.add(this.__linearChange[1]);
				contact.velocity.add(contact.toWorld.transformTransposeVector(delta_velocity));
			}

			if (contact.bodies[1] !== null && contact.bodies[1] !== undefined) {
				if (contact.bodies[1] === chosen_contact.bodies[0]) {
					delta_velocity = this.__angularChange[0].crossProduct(contact.relativeContactPositions[1], this.__helperVector);
					delta_velocity.add(this.__linearChange[0]);
					contact.velocity.substract(contact.toWorld.transformTransposeVector(delta_velocity));
				}

				if (contact.bodies[1] === chosen_contact.bodies[1]) {
					delta_velocity = this.__angularChange[1].crossProduct(contact.relativeContactPositions[1], this.__helperVector);
					delta_velocity.add(this.__linearChange[1]);
					contact.velocity.substract(contact.toWorld.transformTransposeVector(delta_velocity));
				}
			}

			contact.calculateDesiredDeltaVelocity(duration);
		}
	
		iterations_used += 1;
	}
};

module.exports = ContactResolver;