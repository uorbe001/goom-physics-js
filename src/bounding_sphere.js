if (typeof define !== 'function') {
	var define = require('amdefine')(module);
}

define(["goom-math"], function(Mathematics) {
	/**
		Creates a BoundingSphere.
		@class A BoundingSphere is a kind of Bounding Volume used for broad-phase collision detection in Bounding Volume Hierarchies.
		@exports BoundingSphere as Physics.BoundingSphere
		@param {Mathematics.Vector3D} position The origin position of the sphere.
		@param {Number} [radious=1] The radious of the sphere.
		@property {Mathematics.Vector3D} position The origin position of the sphere.
		@property {Number} radious The radious of the sphere.
		@property {Mathematics.Vector3D} __helperVector This vector is used in a few functions as a local variable,
			it is defined as a class variable to avoid the creation of objects at runtime.
	*/
	var BoundingSphere = (function() {
		function BoundingSphere(position, radious) {
			if (position === null || position === undefined) position = new Mathematics.Vector3D();
			this.radious = radious !== null  && radious !== undefined? radious : 1;
			this.position = position.clone();
			this.__helperVector = new Mathematics.Vector3D();
		}

		/**
			Creates a BoundingSphere to enclose the two given spheres.
			@param {Physiscs.BoundingSphere} sphere_one The first bounding sphere to enclose.
			@param {Physiscs.BoundingSphere} sphere_two The second bounding sphere to enclose.
			@returns {Physics.BoundingSphere} The constructed BoundingSphere.
		*/
		BoundingSphere.createfromSpheres = function(sphere_one, sphere_two) {
			var position, radious;
			var center_offset = new Mathematics.Vector3D();
			sphere_two.position.substract(sphere_one.position, center_offset);
			var distance = center_offset.squaredMagnitude();
			var radious_diff = sphere_two.radious - sphere_one.radious;

			if ((radious_diff * radious_diff) >= distance) {
				if (sphere_one.radious > sphere_two.radious) {
					position = sphere_one.position.clone();
					radious = sphere_one.radious;
				} else {
					position = sphere_two.position.clone();
					radious = sphere_two.radious;
				}
			} else {
				distance = Math.sqrt(distance);
				radious = (distance + sphere_one.radious + sphere_two.radious) * 0.5;
				position = sphere_one.position.clone();
				if (distance > 0) position.add(center_offset.scale((radious - sphere_one.radious) / distance));
			}

			return new BoundingSphere(position, radious);
		};

		/**
			Makes this BoundingSphere enclose the two given spheres.
			@param {Physiscs.BoundingSphere} sphere_one The first bounding sphere to enclose.
			@param {Physiscs.BoundingSphere} sphere_two The second bounding sphere to enclose.
		*/
		BoundingSphere.prototype.fitSpheres = function(sphere_one, sphere_two) {
			var x = sphere_two.position.x - sphere_one.position.x,
				y = sphere_two.position.y - sphere_one.position.y,
				z = sphere_two.position.z - sphere_one.position.z;
			var distance = x * x + y * y + z * z;
			var radious_diff = sphere_two.radious - sphere_one.radious;

			if ((radious_diff * radious_diff) >= distance) {
				if (sphere_one.radious > sphere_two.radious) {
					sphere_one.position.clone(this.position);
					this.radious = sphere_one.radious;
				} else {
					sphere_two.position.clone(this.position);
					this.radious = sphere_two.radious;
				}
			} else {
				distance = Math.sqrt(distance);
				this.radious = (distance + sphere_one.radious + sphere_two.radious) * 0.5;
				sphere_one.position.clone(this.position);
				if (distance > 0) {
					var s = (this.radious - sphere_one.radious) / distance;
					this.position.x += x * s;
					this.position.y += y * s;
					this.position.z += z * s;
				}
			}
		};

		/**
			Checks wether this sphere and the given sphere are overlapping.
			@param {Physics.BoundingSphere} sphere The sphere to check overlapping against.
			@returns {Boolean} True if the spheres are overlapping, false otherwise.
		*/
		BoundingSphere.prototype.overlaps = function(sphere) {
			var position_diff = new Mathematics.Vector3D();
			var distance_squared = this.position.substract(sphere.position, position_diff).squaredMagnitude();
			return distance_squared < ((this.radious + sphere.radious) * (this.radious + sphere.radious));
		};

		/**
			Returns the volume of this bounding sphere.
			@returns {Number} The volume of the bounding sphere.
		*/
		BoundingSphere.prototype.size = function() {
			return 1.333333 * Math.PI * this.radious * this.radious * this.radious;
		};

		/**
			Returns the growth necessary for the given sphere to fit inside this one.
			@param {Physics.BoundingSphere} sphere The sphere that must fit inside this one.
			@returns {Number} The ammount this sphere must be grown to fit the given sphere inside it.
		*/
		BoundingSphere.prototype.growth = function(sphere) {
			var center_offset = sphere.position.substract(this.position, this.__helperVector);
			var distance = center_offset.squaredMagnitude();
			var radious_diff = sphere.radious - this.radious;
			var radious = 0;

			if ((radious_diff * radious_diff) >= distance) {
				if (this.radious > sphere.radious) {
					radious = this.radious;
				} else {
					radious = sphere.radious;
				}
			} else {
				distance = Math.sqrt(distance);
				radious = (distance + this.radious + sphere.radious) * 0.5;
			}

			return radious * radious - this.radious * this.radious;
		};

		return BoundingSphere;
	})();

	return BoundingSphere;
});