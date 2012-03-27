if (typeof define !== 'function') {
	var define = require('amdefine')(module);
}

define(["goom-math"], function(Mathematics) {
	var __hasProp = Object.prototype.hasOwnProperty, __extends = function(child, parent) {
		for (var key in parent) { if (__hasProp.call(parent, key)) child[key] = parent[key]; }
		function ctor() { this.constructor = child; }
		ctor.prototype = parent.prototype;
		child.prototype = new ctor();
		child.__super__ = parent.prototype;
		return child;
	};

	/**
		@namespace holds all the primitive classes used in the physics engine.
	*/
	var Primitives = {};

	/**
		Creates a new Primitive.
		@class Primitive representing an object's collision boundaries.
		@param {Mathematics.RigidBody} body The body related to this primitive.
		@param {Mathematics.Matrix4D} [offset] The offset of this primitive from the body origin.
		@property {Mathematics.RigidBody} body The body related to this primitive.
		@property {Mathematics.Matrix4D} offset The offset of this primitive from the body origin.
		@property {Mathematics.Matrix4D} transformationMatrix The transform resulting from the combination of the rigid body's transform and the pimitive's offset.
		@exports Primitive as Physics.Primitives.Primitive
	*/
	var Primitive = (function() {
		function Primitive(body, offset) {
			this.body = body;
			if (offset === null || offset === undefined) offset = new Mathematics.Matrix4D();
			this.offset = offset.clone();
			this.transformationMatrix = new Mathematics.Matrix4D();
			if (this.body !== null && this.body !== undefined) {
				this.calculateInternalData();
			} else {
				this.transformationMatrix = this.offset;
			}
		}

		/**
			Calculates this primitive's internal data, just the transformation matrix for now.
		*/
		Primitive.prototype.calculateInternalData = function() {
			if (this.body !== null && this.body !== undefined) this.body.transformationMatrix.clone(this.transformationMatrix);
			this.transformationMatrix.data[12] += this.offset.data[12];
			this.transformationMatrix.data[13] += this.offset.data[13];
			this.transformationMatrix.data[14] += this.offset.data[14];
		};

		/**
			Returns the position vector from the body origin.
			@param {Mathematics.Vector3D} destination The vector where result is stored.
			@returns {Mathematics.Vector3D} The position vector from the origin.
		*/
		Primitive.prototype.position = function(destination) {
			return this.transformationMatrix.axisVector(3, destination);
		};

		/**
			Rerturns the axis vector from the transformation matrix.
			@param {Mathematics.Vector3D} destination The vector where result is stored.
			@returns {Mathematics.Vector3D} axis vector.
		*/
		Primitive.prototype.axisVector = function(index, destination) {
			return this.transformationMatrix.axisVector(index, destination);
		};

		Primitives.Primitive = Primitive;
		return Primitive;
	})();

	/**
		Creates a new Sphere.
		@class Primitive representing an object's collision boundaries. This primitive is a sphere.
		@param {Mathematics.RigidBody} body The body related to this primitive.
		@param {Number} radius The radius of the sphere.
		@param {Mathematics.Matrix4D} [offset] The offset of this primitive from the body origin.
		@property {Mathematics.RigidBody} body The body related to this primitive.
		@property {Number} radious The radious of the sphere.
		@property {Mathematics.Matrix4D} offset The offset of this primitive from the body origin.
		@exports Sphere as Physics.Primitives.Sphere
	*/
	var Sphere = (function() {
		__extends(Sphere, Primitives.Primitive);

		function Sphere(body, radious, offset) {
			this.radious = radious;
			Sphere.__super__.constructor.call(this, body, offset);
		}

		Primitives.Sphere = Sphere;
		return Sphere;
	})();

	/**
		Creates a new Plane.
		@class Primitive representing an object's collision boundaries. This primitive is a plane.
		@param {Mathematics.RigidBody} body The body related to this primitive.
		@param {Mathematics.Vector3D} normal The plane normal.
		@param {Number} [offset=0] The offset of this primitive from the body origin.
		@property {Mathematics.RigidBody} body The body related to this primitive.
		@property {Mathematics.Vector3D} normal The plane normal.
		@property {Number} offset The offset of this primitive from the body origin
		@exports Plane as Physics.Primitives.Plane
	*/
	var Plane = (function() {
		__extends(Plane, Primitives.Primitive);

		function Plane(body, normal, offset) {
			this.body = body;
			this.normal = normal;
			this.offset = offset !== null && offset !== undefined ? offset : 0;
		}

		/**
			Calculates this primitive's internal data, which is nothing for the plane.
		*/
		Plane.prototype.calculateInternalData = function() {};
		
		Primitives.Plane = Plane;
		return Plane;
	})();

	/**
		Creates a Box primitive.
		@class Primitive representing an object's collision boundaries. This primitive is a box.
		@param {Mathematics.RigidBody} body The body related to this primitive.
		@param {Mathematics.Vector3D} halfSize A vector representing the size of the box from the origin on each axis.
		@param {Mathematics.Matrix4D} [offset] The offset of this primitive from the body origin.
		@property {Mathematics.RigidBody} body The body related to this primitive.
		@param {Mathematics.Vector3D} halfSize A vector representing the size of the box from the origin on each axis.
		@property {Mathematics.Matrix4D} offset The offset of this primitive from the body origin.
		@exports Box as Physics.Primitives.Box
	*/
	var Box = (function() {
		__extends(Box, Primitives.Primitive);
		
		function Box(body, half_size, offset) {
			Box.__super__.constructor.call(this, body, offset);
			this.halfSize = half_size.clone();
		}

		Primitives.Box = Box;
		return Box;
	})();

	return Primitives;
});