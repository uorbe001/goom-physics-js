if (typeof define !== 'function') {
	var define = require('amdefine')(module);
}

define(["goom-math", "./primitives"], function(Mathematics, Primitives) {
	/**
		Creates a RigidBody.
		@class This is the basic rigid body class used for physics simulations.
		@exports RigidBody as Physics.RigidBody
		@property {Number} inverseMass The inverse mass of the rigid body.
		@property {Mathematics.Vector3D} position The position of the object in world space.
		@property {Mathematics.Quaternion} orientation Angular orientation of the body.
		@property {Mathematics.Vector3D} velocity The velocity of the rigid body in world space.
		@property {Mathematics.Vector3D} angular_velocity Angular velocity (or rotation) of the body in world space.
		@property {Mathematics.Matrix4D} transformationMatrix The object's transformation matrix. Cached
		for performance reasons.
		@property {Mathematics.Matrix3D} inverseInertiaTensor The inverse of the body's inertia tensor.
		@property {Mathematics.Matrix3D} inverseInertiaTensorWorld The inverse of the body's inertia
		tensor in world coordinates.
		@property {Mathematics.Mathematics.Vector3D} lastFrameAcceleration The acceleration in the previous frame.
		@property {Mathematics.Mathematics.Vector3D} acceleration The acceleration in this frame.
		@property {Mathematics.Vector3D} accumulatedForce The total accumulated forces to this body.
		@property {Mathematics.Vector3D} accumulatedTorque The total accumulated torque to this body.
		@property {Boolean} isAwake Whether the body is awake or not.
		@property {Boolean} canSleep Wether this body is allowed to sleep or not, some bodies can never fall asleep.
		@property {Number} motion Ammount of motion of the body. Used to put bodies to sleep.
		@property {Array} primitives The list of primitives in this body, used for narrow phase collision detection.
		@property {Physics.BoundingVolumeHierarchyNode} boundingVolumeHierarchyNode BVHNodes are used in broad-phase
		collision detection, rigid bodies will hold a reference to the one holding them in order to notify them on position changes.
		@property {Mathematics.Vector3D} __helperVector This vector is used in a few functions as a local variable,
			it is defined as a class variable to avoid the creation of objects at runtime.
		@param {Mathematics.Vector3D} [position=(0,0,0)] The position of the object in world space.
		@param {Mathematics.Quaternion} [orientation=(1,0,0,0)] Angular orientation of the body.
		@param {Mathematics.Vector3D} [velocity=(0,0,0)] The velocity of the rigid body in world space.
		@param {Mathematics.Vector3D} [angular_velocity=(0,0,0)] Angular velocity (or rotation) of the body in world space.
	*/
	var RigidBody = (function() {
		function RigidBody(position, orientation, velocity, angular_velocity) {
			this.position = position !== null && position !== undefined? position: new Mathematics.Vector3D();
			this.orientation = orientation !== null && position !== undefined? orientation: new Mathematics.Quaternion();
			this.velocity = velocity !== null && velocity !== undefined? velocity: new Mathematics.Vector3D();
			this.angular_velocity = angular_velocity !== null && angular_velocity !== undefined? angular_velocity: new Mathematics.Vector3D();
			this.inverseMass = 0;
			this.acceleration = new Mathematics.Vector3D();
			this.lastFrameAcceleration = new Mathematics.Vector3D();
			this.transformationMatrix = new Mathematics.Matrix4D();
			this.inverseInertiaTensor = new Mathematics.Matrix3D();
			this.inverseInertiaTensorWorld = new Mathematics.Matrix3D();
			this.accumulatedForce = new Mathematics.Vector3D();
			this.accumulatedTorque = new Mathematics.Vector3D();
			this.isAwake = true;
			this.canSleep = true;
			this.motion = 0;
			this.boundingVolumeHierarchyNode = null;
			this.primitives = [];
			this.calculateInternalData();
			this.__helperVector = new Mathematics.Vector3D();
		}

		/**
			Sets the inertia tensor for this body, stored in the inverse inertia tensor
			variable.
			@param {Mathematics.Matrix3D} inertia_tensor The inertia tensor for this body.
		*/
		RigidBody.prototype.setInertiaTensor = function(inertia_tensor) {
			inertia_tensor.inverse(this.inverseInertiaTensor);
		};

		/**
			Sets the inertia tensor's coefficients to this body's inertial tensor.
			@param {Number} coeff00 The coefficient for the 00 element of the inertia tensor.
			@param {Number} coeff11 The coefficient for the 11 element of the inertia tensor.
			@param {Number} coeff22 The coefficient for the 22 element of the inertia tensor.
		*/
		RigidBody.prototype.setInertiaTensorCoefficients = function(coeff00, coeff11, coeff22) {
			var inertia_tensor = new Mathematics.Matrix3D();
			inertia_tensor.setDiagonal(coeff00, coeff11, coeff22);
			inertia_tensor.inverse(this.inverseInertiaTensor);
		};

		/**
			Calculates the internal data from the state data. Should be called after the
			body's state is altered directly.
		*/
		RigidBody.prototype.calculateInternalData = function() {
			this.orientation.normalize();
			//Calculate the transform matrix for this body.
			this.transformationMatrix.makeFromPositionAndOrientation(this.position, this.orientation);
			//Calculate the inertia tensor in world space.
			this.transformationMatrix.transformMatrix3D(this.inverseInertiaTensor, this.inverseInertiaTensorWorld);
			//Update the primitives in this body.
			_ref = this.primitives;
			for (var i = 0, len = _ref.length; i < len; i++) {
				primitive = _ref[i];
				primitive.calculateInternalData();
			}

			//Notify the BVH it needs to be updated.
			if (this.boundingVolumeHierarchyNode !== null) this.boundingVolumeHierarchyNode.updateHierarchy();
		};

		/*
			Wakes this body up.
		*/
		RigidBody.prototype.wakeUp = function() {
			this.isAwake = true;
			this.motion = 2 * RigidBody.SLEEP_EPSILON;
		};

		/*
			Make this body go to sleep.
		*/
		RigidBody.prototype.sleep = function() {
			this.isAwake = false;
			this.velocity.zero();
			this.angular_velocity.zero();
		};

		/**
			Applies a force expressed in world coordinates to this body.
			@param {Mathematics.Vector3D} force The force to be applied to this body.
		*/
		RigidBody.prototype.applyForce = function(force) {
			this.accumulatedForce.add(force);
			this.wakeUp();
		};

		/**
			Applies a force to the given point in the body, both the force and the point are
			expected to be given in world coordinates. Because the force is not applied to
			the center of gravity, it may be split into a force and a torque.
			@param {Mathematics.Vector3D} force The force to be applied to this body.
			@param {Mathematics.Vector3D} point The point in world coordinates where the force is applied.
		*/
		RigidBody.prototype.applyForceAtPoint = function(force, point) {
			point.substract(this.position, this.__helperVector);
			this.accumulatedForce.add(force);
			this.accumulatedTorque.add(this.__helperVector.crossProduct(force));
			this.wakeUp();
		};

		/**
			Applies a force to the gien point in the body, both the force is expected to be
			given in world coordinates, but the point in object coordinates.
			@param {Mathematics.Vector3D} force The force to be applied to this body.
			@param {Mathematics.Vector3D} point The point in object coordinates where the force is applied.
		*/
		RigidBody.prototype.applyForceAtBodyPoint = function(force, point) {
			this.applyForceAtPoint(force, this.transformationMatrix.transformVector(point, this.__helperVector));
		};

		/**
			Applies a torque expressed in world coordinates to this body.
			@param {Mathematics.Vector3D} torque The torque to be applied to this body.
		*/
		RigidBody.prototype.applyTorque = function(torque) {
			this.accumulatedTorque.add(torque);
			this.wakeUp();
		};

		/**
			Clears the total accumulated forces and torque.
		*/
		RigidBody.prototype.clear = function() {
			this.accumulatedForce.zero();
			this.accumulatedTorque.zero();
		};

		/**
			Retuns whether this body has a finite mass or not.
			@returns {Boolean} true if it has a finite mass, false otherwise.
		*/
		RigidBody.prototype.hasFiniteMass = function() {
			return this.inverseMass >= 0;
		};

		/**
			Returns the mass of this body.
			@returns {Number} mass of this body.
		*/
		RigidBody.prototype.getMass = function() {
			if (this.inverseMass === 0) return Number.MAX_VALUE;
			return 1 / this.inverseMass;
		};

		/**
			Sets the mass of this body.
			@param {Number} mass The mass of this body.
		*/
		RigidBody.prototype.setMass = function(mass) {
			if (mass === 0) this.inverseMass = Number.MAX_VALUE;
			this.inverseMass = 1 / mass;
		};

		/**
			Adds and creates a primitive from the given description.
			@param primitive_data A json object or an array of json objects describing the primitive to be added.
			@throws An exception when the primitive type is not known.
		*/
		RigidBody.prototype.addPrimitives = function(primitive_data) {
			//If Primitives is an array, it means we have to add more than one primitive, otherwise, just one primitive.
			if (!(primitive_data instanceof Array)) {
				switch (primitive_data.type.toLowerCase()) {
					case 'sphere':
						this.primitives.push(new Primitives.Sphere(this, primitive_data.radious, primitive_data.offset !== null && primitive_data.offset !== undefined? (new Mathematics.Matrix4D()).set(primitive_data.offset): null));
						break;
					case 'box':
						this.primitives.push(new Primitives.Box(this, new Mathematics.Vector3D(primitive_data.halfSize.x, primitive_data.halfSize.y, primitive_data.halfSize.z), primitive_data.offset !== null && primitive_data.offset !== undefined? (new Mathematics.Matrix4D()).set(primitive_data.offset): null));
						break;
					default:
						throw "Unkwnown primitive type";
				}
				return;
			}

			var one_primitive_data, i, len;
			for (i = 0, len = primitive_data.length; i < len; i++) {
				one_primitive_data = primitive_data[i];
				switch (one_primitive_data.type.toLowerCase()) {
					case 'sphere':
						this.primitives.push(new Primitives.Sphere(this, one_primitive_data.radious, one_primitive_data.offset !== null && primitive_data.offset !== undefined? (new Mathematics.Matrix4D()).set(one_primitive_data.offset) : null));
						break;
					case 'box':
						this.primitives.push(new Primitives.Box(this, new Mathematics.Vector3D(one_primitive_data.halfSize.x, one_primitive_data.halfSize.y, one_primitive_data.halfSize.z), one_primitive_data.offset !== null && primitive_data.offset !== undefined? (new Mathematics.Matrix4D()).set(one_primitive_data.offset): null));
						break;
					default:
						throw "Unkwnown primitive type";
				}
			}
		};

		/**
			Integrates the body forward in time by the given ammount.
			@param {Number} duration The ammount of time to step forward.
		*/
		RigidBody.prototype.integrate = function(duration) {
			if (!this.isAwake) return;

			//Calculate linear acceleration
			this.acceleration.clone(this.lastFrameAcceleration);
			this.lastFrameAcceleration.add(this.accumulatedForce.scale(this.inverseMass, this.__helperVector));
			//Calculate the angular acceleration from toques.
			this.inverseInertiaTensorWorld.transformVector(this.accumulatedTorque, this.__helperVector);
			//Update velocities
			this.angular_velocity.add(this.__helperVector.scale(duration));
			this.velocity.add(this.lastFrameAcceleration.scale(duration, this.__helperVector));
			//Impose drag
			this.velocity.scale(Math.pow(RigidBody.LINEAR_DAMPING, duration));
			this.angular_velocity.scale(Math.pow(RigidBody.ANGULAR_DAMPING, duration));
			//Clear forces for the next integration step
			this.clear();

			//Update the kinetic energy and possibly put the body to sleep
			if (this.canSleep) {
				var current_motion = this.velocity.squaredMagnitude() + this.angular_velocity.squaredMagnitude();
				var bias = Math.pow(0.5, duration);
				this.motion = bias * this.motion + (1 - bias) * current_motion;

				if (this.motion < RigidBody.SLEEP_EPSILON) {
					this.sleep();
					return;
				} else if (this.motion > 5 * RigidBody.SLEEP_EPSILON) {
					this.motion = 5 * RigidBody.SLEEP_EPSILON;
				} else if (current_motion < RigidBody.SLEEP_EPSILON) {
					return;
				}
			}

			//Update positions
			this.position.add(this.velocity.scale(duration, this.__helperVector));
			this.orientation.addVector(this.angular_velocity.scale(duration, this.__helperVector));
			//Update derived data
			this.calculateInternalData();
		};

		/**
			@static
			This value is used to limit the energy under which a body will be set to sleep.
		*/
		RigidBody.SLEEP_EPSILON = 0.2;

		/**
			@static
			This is the damping coefficient used for linear velocity, it will speed down linear velocity with time.
		*/
		RigidBody.LINEAR_DAMPING = 0.9999;

		/**
			@static
			This is the damping coefficient used for angular velocity, it will speed down angular velocity with time.
		*/
		RigidBody.ANGULAR_DAMPING = 0.9999;
		return RigidBody;
	})();
	return RigidBody;
});