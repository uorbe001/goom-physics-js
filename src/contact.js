if (typeof define !== 'function') {
	var define = require('amdefine')(module);
}

define(["goom-math", "./rigid_body"], function(Mathematics) {
	/**
		Creates a new Contact.
		@class Holds the information required to resolve contacts.
		@param {Mathematics.Vector3D} [point=(0,0,0)] The position of the contact in world coordinates.
		@param {Mathematics.Vector3D} [normal=(0,0,0)] The direction of the contact in world coordinates.
		@param {Number} [penetration=0] The depth of penetration at the contact point.
		@property {Mathematics.Vector3D} point The position of the contact in world coordinates.
		@property {Mathematics.Vector3D} normal The direction of the contact in world coordinates.
		@property {Mathematics.Vector3D} vector The velocity of the contact in world coordinates.
		@property {Number} penetration The depth of penetration at the contact point.
		@property {Array} bodies Holds references to the bodies in contact.
		@property {Number} restitution Restitution coeficient in this contact.
		@property {Number} friction Friction coeficient in this contact.
		@property {Array} relativeContactPositions Array holding two Vectors, each holding the contact position relative to each body.
		@property {Mathematics.Matrix3D} toWorld Matrix holding the contact to world coordinate changes.
		@property {Number} desiredDeltaVelocity The desired change in velocity for contact resolution.
		@exports Contact as Physics.Contact
	*/
	var Contact = (function() {
		function Contact(point, normal, penetration) {
			this.penetration = penetration !== null && penetration !== undefined ? penetration : 0;
			this.point = point === null || point === undefined? new Mathematics.Vector3D() : point.clone();
			this.normal = normal === null || point === undefined ? new Mathematics.Vector3D() : normal.clone();
			this.bodies = new Array(2);
			this.relativeContactPositions = new Array(2);
			this.relativeContactPositions[0] = new Mathematics.Vector3D();
			this.relativeContactPositions[1] = new Mathematics.Vector3D();
			this.desiredDeltaVelocity = 0;
			this.velocity = new Mathematics.Vector3D();
			this.__helperVector = new Mathematics.Vector3D();
			this.__contactTangent = new Array(2);
			this.__contactTangent[0] = new Mathematics.Vector3D();
			this.__contactTangent[1] = new Mathematics.Vector3D();
			this.toWorld = new Mathematics.Matrix3D();
		}

		/**
			Sets the data related to the bodies in the collision.
			@param {Physics.RigidBody} first_body The first body in the collision.
			@param {Physics.RigidBody} second_body The second body in the collison.
			@param {Number} restitution Restitution coeficient in this contact.
			@param {Number} friction Friction coeficient in this contact.
		*/
		Contact.prototype.setContactData = function(first_body, second_body, restitution, friction) {
			this.bodies[0] = first_body;
			this.bodies[1] = second_body;
			this.restitution = restitution;
			this.friction = friction;
		};

		/**
			Swaps the position of the bodies in this contact, updating the contact normal accordingly.
			@inner
		*/
		Contact.prototype.__swapBodies = function() {
			this.normal.scale(-1);
			var tmp = this.bodies[0];
			this.bodies[0] = this.bodies[1];
			this.bodies[1] = tmp;
		};

		/**
			Updates the awake state of the rigid bodies. A body will be woken up if it is in contact with a body that is awake.
		*/
		Contact.prototype.matchAwakeState = function() {
			//Contacts with the world don't wake up bodies
			if (this.bodies[1] === null || this.bodies[1] === undefined) return;
			//If one of the bodies is awake and the other asleep
			if (this.bodies[0].isAwake ^ this.bodies[1].isAwake) {
				if (this.bodies[0].isAwake) this.bodies[0].wakeUp();
				else this.bodies[1].wakeUp();
			}
		};

		/**
			Calculates the internal value for the desired delta velocity.
			@param {Number} duration The duration of the previous integration step.
		*/
		Contact.prototype.calculateDesiredDeltaVelocity = function(duration) {
			var scaled_contact = this.normal.scale(duration, this.__helperVector);
			var velocity_from_acceleration = 0;

			//Calculate the acceleration induced velocity accumulated this frame
			if (this.bodies[0].isAwake) {
				velocity_from_acceleration = this.bodies[0].lastFrameAcceleration.dotProduct(scaled_contact);
			}

			if ((this.bodies[1] !== null && this.bodies[1] !== undefined) && this.bodies[1].isAwake) {
				velocity_from_acceleration -= this.bodies[1].lastFrameAcceleration.dotProduct(scaled_contact);
			}

			//If velocity is too slow, limit restitution
			var local_restitution = this.restitution;
			if (Math.abs(this.velocity.x) < Contact.MIN_VELOCITY) {
				local_restitution = 0;
			}
			
			//Calculate the bounce velocity with the removed acceleration velocity.
			this.desiredDeltaVelocity = -this.velocity.x - local_restitution * (this.velocity.x - velocity_from_acceleration);
		};

		/**
			Calculate internal data from state data.
			@param {Number} duration The duration of the previous integration step.
		*/
		Contact.prototype.calculateInternalData = function(duration) {
			var s;
			//Make the first body be non-null
			if (this.bodies[0] === null || this.bodies[0]  === undefined) this._swapBodies();
			//Calculate contact basis
			if (Math.abs(this.normal.x) > Math.abs(this.normal.y)) {
				//Scaling factor to ensure results are normalized.
				s = 1 / Math.sqrt(this.normal.z * this.normal.z + this.normal.x * this.normal.x);
				//The new x-axis is at right angles to the world y-axis
				this.__contactTangent[0].x = this.normal.z * s;
				this.__contactTangent[0].y = 0;
				this.__contactTangent[0].z = -this.normal.x * s;
				//The new y-axis is at right angles to the new x and z axes
				this.__contactTangent[1].x = this.normal.y * this.__contactTangent[0].x;
				this.__contactTangent[1].y = this.normal.z * this.__contactTangent[0].x - this.normal.x * this.__contactTangent[0].z;
				this.__contactTangent[1].z = -this.normal.y * this.__contactTangent[0].x;
			} else {
				//Scaling factor to ensure results are normalized.
				s = 1 / Math.sqrt(this.normal.z * this.normal.z + this.normal.y * this.normal.y);
				//The new x-axis is at right angles to the world x-axis
				this.__contactTangent[0].x = 0;
				this.__contactTangent[0].y = -this.normal.z * s;
				this.__contactTangent[0].z = this.normal.y * s;
				//The new y-axis is at right angles to the new x and z axes
				this.__contactTangent[1].x = this.normal.y * this.__contactTangent[0].z - this.normal.z * this.__contactTangent[0].y;
				this.__contactTangent[1].y = -this.normal.x * this.__contactTangent[0].z;
				this.__contactTangent[1].z = this.normal.x * this.__contactTangent[0].y;
			}

			this.toWorld.makeFromVectors(this.normal, this.__contactTangent[0], this.__contactTangent[1]);
			//Store the position of the contact relative to each body
			this.point.substract(this.bodies[0].position, this.relativeContactPositions[0]);
			if (this.bodies[1] !== null && this.bodies[1] !== undefined) {
				this.point.substract(this.bodies[1].position, this.relativeContactPositions[1]);
			}

			//Calculate relative velocity of the bodies at contact point
			this.relativeContactPositions[0].crossProduct(this.bodies[0].angular_velocity, this.velocity);
			this.velocity.add(this.bodies[0].velocity);
			//Calculate the ammount of velocity due to forces without reactions
			var acceleration_velocity = this.bodies[0].lastFrameAcceleration.scale(duration, this.__helperVector);
			//We are only interested in planar acceleration, so we ignore acceleration in the contact normal direction
			acceleration_velocity.x = 0;
			this.velocity.add(acceleration_velocity);

			if (this.bodies[1] !== null && this.bodies[1] !== undefined) {
				//Do the same for the second body
				var second_velocity = this.relativeContactPositions[1].crossProduct(this.bodies[1].angular_velocity, this.__helperVector);
				second_velocity.add(this.bodies[1].velocity);
				//Add the velocity due to the second body to the total contact velocity
				this.velocity.substract(second_velocity);
				//Calculate the ammount of velocity due to forces without reactions
				acceleration_velocity = this.bodies[1].lastFrameAcceleration.scale(duration, this.__helperVector);
				//We are only interested in plannar acceleration, so we ignore acceleration in the contact normal direction
				acceleration_velocity.x = 0;
				//Add the velocity due to the second body's acceleration to the total contact velocity
				this.velocity.substract(acceleration_velocity);
			}

			//Convert velocity to contact coordinates
			this.toWorld.transformTransposeVector(this.velocity);
			//Calculate the desired change in velocity for resolution
			this.calculateDesiredDeltaVelocity(duration);
		};

		/**
			Performs a inertia weighted penetration resolution of this contact.
			@param {Array} linear_change Array holding vectors representing the linear position change for each body.
			@param {Array} angular_change Array holding vectors representing the angular position change for each body.
			@param {Number} penetration Ammount of penetration to resolve.
		*/
		/*Contact.prototype.resolvePosition = function(linear_change, angular_change, penetration) {
		var angular_inertia, angular_inertia_world, angular_move, linear_inertia, linear_move, max_magnitude, projection, target_angular_direction, total_inertia, total_move;
		total_inertia = 0;
		angular_inertia = [];
		linear_inertia = [];
		angular_inertia_world = this.normal.crossProduct(this.relativeContactPositions[0], new Math.Vector3D());
		angular_inertia_world.transformByMatrix(this.bodies[0].inverseInertialTensorWorld);
		this.relativeContactPositions[0].crossProduct(angular_inertia_world, angular_inertia_world);
		angular_inertia.push(angular_inertia_world.dotProduct(this.normal));
		linear_inertia.push(this.bodies[0].inverseMass);
		total_inertia += angular_inertia[0] + linear_inertia[0];
		if (this.bodies[1] != null) {
		angular_inertia_world = this.normal.crossProduct(this.relativeContactPositions[1], new Math.Vector3D());
		angular_inertia_world.transformByMatrix(this.bodies[1].inverseInertialTensorWorld);
		this.relativeContactPositions[1].crossProduct(angular_inertia_world, angular_inertia_world);
		angular_inertia.push(angular_inertia_world.dotProduct(this.normal));
		linear_inertia.push(this.bodies[1].inverseMass);
		total_inertia += angular_inertia[1] + linear_inertia[1];
		}
		angular_move = [];
		linear_move = [];
		angular_move.push(penetration * (angular_inertia[0] / total_inertia));
		linear_move.push(penetration * (linear_inertia[0] / total_inertia));
		projection = (this.normal.scale(-this.relativeContactPositions[0].dotProduct(this.normal), new Math.Vector3D())).add(this.relativeContactPositions[0]);
		max_magnitude = Contact.ANGULAR_LIMIT * projection.magnitude();
		if (angular_move[0] < -max_magnitude) {
		total_move = angular_move[0] + linear_move[0];
		angular_move[0] = -max_magnitude;
		linear_move[0] = total_move - angular_move[0];
		} else if (angular_move[0] > max_magnitude) {
		total_move = angular_move[0] + linear_move[0];
		angular_move[0] = max_magnitude;
		linear_move[0] = total_move - angular_move[0];
		}
		if (angular_move[0] === 0) {
		angular_change[0].zero();
		} else {
		target_angular_direction = this.normal.crossProduct(this.relativeContactPositions[0], new Math.Vector3D());
		angular_change[0] = target_angular_direction.transformByMatrix(this.bodies[0].inverseInertialTensorWorld).scale(angular_move[0] / angular_inertia[0]);
		}
		linear_change[0] = this.normal.scale(linear_move[0], new Math.Vector3D());
		this.bodies[0].position.add(linear_change[0]);
		this.bodies[0].orientation.addVector(angular_change[0]);
		if (!this.bodies[0].isAwake) {
		this.bodies[0].calculateInternalData();
		}
		if (this.bodies[1] != null) {
		angular_move.push(-penetration * (angular_inertia[1] / total_inertia));
		linear_move.push(-penetration * (linear_inertia[1] / total_inertia));
		projection = (this.normal.scale(-this.relativeContactPositions[1].dotProduct(this.normal), new Math.Vector3D())).add(this.relativeContactPositions[1]);
		max_magnitude = Contact.ANGULAR_LIMIT * projection.magnitude();
		if (angular_move[1] < -max_magnitude) {
		total_move = angular_move[1] + linear_move[1];
		angular_move[1] = -max_magnitude;
		linear_move[1] = total_move - angular_move[1];
		} else if (angular_move[1] > max_magnitude) {
		total_move = angular_move[1] + linear_move[1];
		angular_move[1] = max_magnitude;
		linear_move[1] = total_move - angular_move[1];
		}
		if (angular_move[1] === 0) {
		angular_change[1].zero();
		} else {
		target_angular_direction = this.normal.crossProduct(this.relativeContactPositions[1], new Math.Vector3D());
		angular_change[1] = target_angular_direction.transformByMatrix(this.bodies[1].inverseInertialTensorWorld).scale(angular_move[1] / angular_inertia[1]);
		}
		linear_change[1] = this.normal.scale(linear_move[1], new Math.Vector3D());
		this.bodies[1].position.add(linear_change[1]);
		this.bodies[1].orientation.addVector(angular_change[1]);
		if (!this.bodies[1].isAwake) {
		this.bodies[1].calculateInternalData();
		}
		}
		};*/
		/**
		Performs a inertia weighted impulse resolution of this contact.
		@param {Array} velocity_change Array holding the velocity change for each body.
		@param {Array} rotation_change Array holding the rotation change for each body.
		*/
		/*Contact.prototype.resolveVelocity = function(velocity_change, rotation_change) {
		var delta_velocity, delta_velocity_world, impulse, impulsive_torque;
		impulse = new Math.Vector3D();
		if (this.friction === 0) {
		delta_velocity_world = this.normal.crossProduct(this.relativeContactPositions[0], new Math.Vector3D());
		delta_velocity_world.transformByMatrix(this.bodies[0].inverseInertialTensorWorld);
		this.relativeContactPositions[0].crossProduct(delta_velocity_world, delta_velocity_world);
		delta_velocity = delta_velocity_world.dotProduct(this.normal);
		delta_velocity += this.bodies[0].inverseMass;
		if (this.bodies[1] != null) {
		delta_velocity_world = this.normal.crossProduct(this.relativeContactPositions[1], new Math.Vector3D());
		delta_velocity_world.transformByMatrix(this.bodies[1].inverseInertialTensorWorld);
		this.relativeContactPositions[1].crossProduct(delta_velocity_world, delta_velocity_world);
		delta_velocity += delta_velocity_world.dotProduct(this.normal);
		delta_velocity += this.bodies[1].inverseMass;
		}
		impulse.set(this.desiredDeltaVelocity / delta_velocity, 0, 0);
		}

		impulse.transformByMatrix(this.toWorld);
		impulsive_torque = impulse.crossProduct(this.relativeContactPositions[0], new Math.Vector3D());
		impulsive_torque.transformByMatrix(this.bodies[0].inverseInertialTensorWorld, rotation_change[0]);
		impulse.scale(this.bodies[0].inverseMass, velocity_change[0]);
		this.bodies[0].rotation.add(rotation_change[0]);
		this.bodies[0].velocity.add(velocity_change[0]);
		if (this.bodies[1] != null) {
		impulse.scale(-1);
		impulse.crossProduct(this.relativeContactPositions[1], impulsive_torque);
		impulsive_torque.transformByMatrix(this.bodies[1].inverseInertialTensorWorld, rotation_change[1]);
		impulse.scale(this.bodies[1].inverseMass, velocity_change[1]);
		this.bodies[1].rotation.add(rotation_change[1]);
		this.bodies[1].velocity.add(velocity_change[1]);
		}

		};*/
		
		/**
			@static
			The miniumum velocity for contact resolution, if the velocity is smaller restitution will be limited.
		*/
		Contact.MIN_VELOCITY = 0.25;
		/**
			@static
			Angular limit used for angular position resolution.
		*/
		Contact.ANGULAR_LIMIT = 0.2;
		return Contact;
	})();
	return Contact;
});