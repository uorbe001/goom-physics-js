if (typeof define !== 'function') {
	var define = require('amdefine')(module);
}

define(["goom-math", "./contact", "./intersection_tests", "./primitives","./rigid_body"], function(Mathematics, Contact, IntersectionTests) {
	/**
		Creates a new CollisionDetector.
		@class Collision detectors are used to check collisions between primitive and fetch the contact data.
		@param {Number} cache_limit The max size of the cache for contacts, cache_limit contacts will be created
			when the constructor is called, and they will be reused throught time. When the contact number is over
			the limit, new contacts will be instantiated but not cached.
		@property {Number} MAX_CONTACTS The max size of the cache for contacts.
		@property {Array} contactCache An array holding the cached contacts.

		@exports CollisionDetector as Physics.CollisionDetector
	*/
	var CollisionDetector = (function() {
		function CollisionDetector(cache_limit) {
			this.MAX_CONTACTS = cache_limit === null || cache_limit === undefined ? 30: cache_limit;
			this.contactCache = new Array(this.MAX_CONTACTS);
			this.contactCacheTop = 0;
			//Initialize the contact cache.
			for (var i = this.MAX_CONTACTS; i >= 0; i--) {
				this.contactCache[i] = new Contact();
			}
			//Helper data used to avoid instantiating objects at runtime.
			this.__helperVector = new Mathematics.Vector3D();
			this.__helperVector2 = new Mathematics.Vector3D();
			this.__helperMatrix = new Mathematics.Matrix3D();
			this.__axisTestData = new Array(3);
		}

		//We will need to go through each combination of vectors, and it's best not to create this on the fly.
		CollisionDetector.multipliers = [[1, 1, 1], [-1, 1, 1], [1, -1, 1], [-1, -1, 1], [1, 1, -1], [-1, 1, -1], [1, -1, -1], [-1, -1, -1]];

		/**
			Returns the last free contact in the cache if there is any, and creates a new contact when there isn't.
			@inner
		*/
		CollisionDetector.prototype.__getFreeContact = function() {
			//When there is no contacts free, instantiate a new one.
			if (this.contactCacheTop >= this.MAX_CONTACTS) return new Contact();
			//Return the last empty contact from cache.
			this.contactCacheTop++;
			return this.contactCache[this.contactCacheTop - 1];
		};

		/**
			Sets the contact cache to the initial status, reusing the cached contacts.
		*/
		CollisionDetector.prototype.clearCache = function() {
			this.contactCacheTop = 0;
		};

		/**
			Checks for contacts between two spheres.
			@param {Physics.Sphere} sphere_one The first sphere to check.
			@param {Physics.Sphere} sphere_two The second sphere to check.
			@param data Data affecting the contact resolution.
			@param {Array} contacts The array where contact data will be stored.
			@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.sphereAndSphere = function(sphere_one, sphere_two, data, contacts) {
			//Cache sphere positions
			var position_one = sphere_one.position(this.__helperVector);
			var position_two = sphere_two.position(this.__helperVector2);
			//Find the vector between the objects
			var midline = position_one.substract(position_two, this.__helperVector2);
			var size = midline.magnitude();
			//Check if we are too far away
			if ((size <= 0) || (size >= (sphere_one.radious + sphere_two.radious))) return false;
			
			//Prepare the contact.
			var contact = this.__getFreeContact();
			//Manually create the normal
			midline.scale(1 / size, contact.normal);
			position_one.substract(midline.scale(0.5), contact.point);
			contact.penetration = sphere_one.radious + sphere_two.radious - size;
			contact.setContactData(sphere_one.body, sphere_two.body, data.restitution, data.friction);
			contacts.push(contact);
			return true;
		};

		/**
			Checks for contacts between a sphere and a half-plane(a plane with infinite solid behind de plane).
			@param {Physics.Sphere} sphere The sphere to check.
			@param {Physics.Plane} plane The plane to check.
			@param data Data affecting the contact resolution.
			@param {Array} contacts The array where contact data will be stored.
			@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.sphereAndHalfSpace = function(sphere, plane, data, contacts) {
			var sphere_position = sphere.position(this.__helperVector);
			var distance = plane.normal.dotProduct(sphere_position) - sphere.radious - plane.offset;
			//Check if we are too far away
			if (distance >= 0) return false;
			//Prepare the contact.
			var contact = this.__getFreeContact();
			sphere_position.substract(plane.normal.scale(distance + sphere.radious, contact.point), contact.point);
			plane.normal.clone(contact.normal);
			contact.penetration = -distance;
			contact.setContactData(sphere.body, null, data.restitution, data.friction);
			contacts.push(contact);
			return true;
		};

		/**
			Checks for contacts between a box and a half space.
			@param {Physics.Box} box The box to check.
			@param {Physics.Plane} plane The plane to check.
			@param data Data affecting the contact resolution.
			@param {Array} contacts The array where contact data will be stored.
			@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.boxAndHalfSpace = function(box, plane, data, contacts) {
			//Early-out check for intersection, performance improvement
			if (!IntersectionTests.boxAndHalfSpace(box, plane)) return false;
			
			contacts_found = false;
			//Go through the different vertices
			for (i = 0; i <= 7; i++) {
				var vertex_position = this.__helperVector.set(CollisionDetector.multipliers[i][0], CollisionDetector.multipliers[i][1], CollisionDetector.multipliers[i][2]);
				vertex_position.componentProduct(box.halfSize);
				box.transformationMatrix.transformVector(vertex_position);
				//Calculate the distance from the plane
				var vertex_distance = vertex_position.dotProduct(plane.normal);
				if (vertex_distance <= plane.offset) {
					//Create contact data
					var contact = this.__getFreeContact();
					//The point is halfway between the vertex and the point.
					plane.normal.scale(-(vertex_distance - plane.offset), contact.point);
					contact.point.add(vertex_position);
					plane.normal.clone(contact.normal);
					contact.penetration = plane.offset - vertex_distance;
					contact.setContactData(box.body, null, data.restitution, data.friction);
					contacts.push(contact);
					contacts_found = true;
				}
			}
			return contacts_found;
		};

		/**
			Checks for contacts between a box and a sphere.
			@param {Physics.Box} box The box to check.
			@param {Physics.Sphere} sphere The sphere to check.
			@param data Data affecting the contact resolution.
			@param {Array} contacts The array where contact data will be stored.
			@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.boxAndSphere = function(box, sphere, data, contacts) {
			//We need the center of the sphere to be in box coordinates
			var sphere_center = sphere.position(this.__helperVector);
			var relative_center = box.transformationMatrix.transformInverseVector(sphere_center, this.__helperMatrix);

			//Early-out check
			if (((Math.abs(relative_center.x) - sphere.radious) > box.halfSize.x) ||
				((Math.abs(relative_center.y) - sphere.radious) > box.halfSize.y) ||
				((Math.abs(relative_center.z) - sphere.radious) > box.halfSize.z)) {

				return false;
			}

			//Clamp each coordinate to the box
			var distance = relative_center.x;
			if (distance > box.halfSize.x) distance = box.halfSize.x;
			if (distance < -box.halfSize.x) distance = -box.halfSize.x;
			this.__helperVector2.x = distance;

			distance = relative_center.y;
			if (distance > box.halfSize.y) distance = box.halfSize.y;
			if (distance < -box.halfSize.y) distance = -box.halfSize.y;
			this.__helperVector2.y = distance;
			
			distance = relative_center.z;
			if (distance > box.halfSize.z) distance = box.halfSize.z;
			if (distance < -box.halfSize.z) distance = -box.halfSize.z;
			this.__helperVector2.z = distance;
			
			distance = this.__helperVector2.substract(relative_center, this.__helperVector).squaredMagnitude();
			if (distance > (sphere.radious * sphere.radious)) return false;
			
			sphere_center = sphere.position(this.__helperVector);
			//Create contact data
			var contact = this.__getFreeContact();
			box.transformationMatrix.transformVector(this.__helperVector2);
			this.__helperVector2.clone(contact.point);
			this.__helperVector2.substract(sphere_center).normalize().clone(contact.normal);
			contact.penetration = sphere.radious - Math.sqrt(distance);
			contact.setContactData(box.body, sphere.body, data.restitution, data.friction);
			contacts.push(contact);
			return true;
		};

		/**
			If the given boxes collide and the penetration on the requested axis is smaller than the previously smallest,
			it updates these values.
			@inner
		*/
		CollisionDetector.prototype._testAxis = function(box_one, box_two, axis, to_centre, index, smallest_penetration, smallest_case) {
			//Don't check almost parallel axes
			if (axis.squaredMagnitude() < 0.0001) {
				this.__axisTestData[0] = true, this.__axisTestData[1] = smallest_penetration, this.__axisTestData[2] = smallest_case;
				return this.__axisTestData;
			}

			//Get the penetration
			axis.normalize();
			var projection_one = box_one.halfSize.x * Math.abs(axis.dotProduct(box_one.axisVector(0, this.__helperVector))) +
								box_one.halfSize.y * Math.abs(axis.dotProduct(box_one.axisVector(1, this.__helperVector))) +
								box_one.halfSize.z * Math.abs(axis.dotProduct(box_one.axisVector(2, this.__helperVector)));
			var projection_two = box_two.halfSize.x * Math.abs(axis.dotProduct(box_two.axisVector(0, this.__helperVector))) +
								box_two.halfSize.y * Math.abs(axis.dotProduct(box_two.axisVector(1, this.__helperVector))) +
								box_two.halfSize.z * Math.abs(axis.dotProduct(box_two.axisVector(2, this.__helperVector)));
			var penetration = projection_one + projection_two - Math.abs(to_centre.dotProduct(axis));
		
			if (penetration < 0) {
				this.__axisTestData[0] = false, this.__axisTestData[1] = smallest_penetration, this.__axisTestData[2] = smallest_case;
				return this.__axisTestData;
			}

			//Did we find a contact with smaller penetration?
			if (penetration < smallest_penetration) {
				smallest_penetration = penetration;
				smallest_case = index;
			}
			
			this.__axisTestData[0] = true, this.__axisTestData[1] = smallest_penetration, this.__axisTestData[2] = smallest_case;
			return this.__axisTestData;
		};

		/**
		Checks for contacts between two boxes.
		@param {Physics.Box} box_one The first box to check.
		@param {Physics.Box} box_two The second box to check.
		@param data Data affecting the contact resolution.
		@param {Array} contacts The array where contact data will be stored.
		@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.boxAndBox = function(box_one, box_two, data, contacts) {
		var axis, axis_name, best, best_single_axis, box_one_axis, box_one_axis_index, box_one_axis_name, box_two_axis, box_two_axis_index, box_two_axis_name, contact, denominator, dp_one_two, dp_st_one, dp_st_two, i, mua, mub, normal, penetration, point_on_box_one_edge, point_on_box_two_edge, sm_one, sm_two, test, to_centre, to_st, use_one, vertex, _ref, _ref10, _ref11, _ref12, _ref13, _ref14, _ref15, _ref2, _ref3, _ref4, _ref5, _ref6, _ref7, _ref8, _ref9;
		to_centre = box_two.position().substract(box_one.position());
		penetration = best = Number.MAX_VALUE;
		test = false;
		_ref = this._testAxis(box_one, box_two, box_one.axisVector(0), to_centre, 0, penetration, best), test = _ref[0], penetration = _ref[1], best = _ref[2];
		if (!test) {
		return false;
		}
		_ref2 = this._testAxis(box_one, box_two, box_one.axisVector(1), to_centre, 1, penetration, best), test = _ref2[0], penetration = _ref2[1], best = _ref2[2];
		if (!test) {
		return false;
		}
		_ref3 = this._testAxis(box_one, box_two, box_one.axisVector(2), to_centre, 2, penetration, best), test = _ref3[0], penetration = _ref3[1], best = _ref3[2];
		if (!test) {
		return false;
		}
		_ref4 = this._testAxis(box_one, box_two, box_two.axisVector(0), to_centre, 3, penetration, best), test = _ref4[0], penetration = _ref4[1], best = _ref4[2];
		if (!test) {
		return false;
		}
		_ref5 = this._testAxis(box_one, box_two, box_two.axisVector(1), to_centre, 4, penetration, best), test = _ref5[0], penetration = _ref5[1], best = _ref5[2];
		if (!test) {
		return false;
		}
		_ref6 = this._testAxis(box_one, box_two, box_two.axisVector(2), to_centre, 5, penetration, best), test = _ref6[0], penetration = _ref6[1], best = _ref6[2];
		if (!test) {
		return false;
		}
		best_single_axis = best;
		_ref7 = this._testAxis(box_one, box_two, box_two.axisVector(0).crossProduct(box_one.axisVector(0)), to_centre, 6, penetration, best), test = _ref7[0], penetration = _ref7[1], best = _ref7[2];
		if (!test) {
		return false;
		}
		_ref8 = this._testAxis(box_one, box_two, box_two.axisVector(1).crossProduct(box_one.axisVector(0)), to_centre, 7, penetration, best), test = _ref8[0], penetration = _ref8[1], best = _ref8[2];
		if (!test) {
		return false;
		}
		_ref9 = this._testAxis(box_one, box_two, box_two.axisVector(2).crossProduct(box_one.axisVector(0)), to_centre, 8, penetration, best), test = _ref9[0], penetration = _ref9[1], best = _ref9[2];
		if (!test) {
		return false;
		}
		_ref10 = this._testAxis(box_one, box_two, box_two.axisVector(0).crossProduct(box_one.axisVector(1)), to_centre, 9, penetration, best), test = _ref10[0], penetration = _ref10[1], best = _ref10[2];
		if (!test) {
		return false;
		}
		_ref11 = this._testAxis(box_one, box_two, box_two.axisVector(1).crossProduct(box_one.axisVector(1)), to_centre, 10, penetration, best), test = _ref11[0], penetration = _ref11[1], best = _ref11[2];
		if (!test) {
		return false;
		}
		_ref12 = this._testAxis(box_one, box_two, box_two.axisVector(2).crossProduct(box_one.axisVector(1)), to_centre, 11, penetration, best), test = _ref12[0], penetration = _ref12[1], best = _ref12[2];
		if (!test) {
		return false;
		}
		_ref13 = this._testAxis(box_one, box_two, box_two.axisVector(0).crossProduct(box_one.axisVector(2)), to_centre, 12, penetration, best), test = _ref13[0], penetration = _ref13[1], best = _ref13[2];
		if (!test) {
		return false;
		}
		_ref14 = this._testAxis(box_one, box_two, box_two.axisVector(1).crossProduct(box_one.axisVector(2)), to_centre, 13, penetration, best), test = _ref14[0], penetration = _ref14[1], best = _ref14[2];
		if (!test) {
		return false;
		}
		_ref15 = this._testAxis(box_one, box_two, box_two.axisVector(2).crossProduct(box_one.axisVector(2)), to_centre, 14, penetration, best), test = _ref15[0], penetration = _ref15[1], best = _ref15[2];
		if (!test) {
		return false;
		}
		if (best < 3) {
		normal = box_one.axisVector(best);
		if (normal.dotProduct(to_centre) > 0) {
		normal.scale(-1);
		}
		vertex = box_two.halfSize.clone();
		if (box_two.axisVector(0).dotProduct(normal) < 0) {
		vertex.x = -vertex.x;
		}
		if (box_two.axisVector(1).dotProduct(normal) < 0) {
		vertex.y = -vertex.y;
		}
		if (box_two.axisVector(2).dotProduct(normal) < 0) {
		vertex.z = -vertex.z;
		}
		contact = new Physics.Contact(vertex.transformByMatrix(box_two.transformationMatrix), normal, penetration);
		contact.setContactData(box_one.body, box_two.body, data.restitution, data.friction);
		contacts.push(contact);
		return true;
		} else if (best < 6) {
		best -= 3;
		to_centre.scale(-1);
		normal = box_two.axisVector(best);
		if (normal.dotProduct(to_centre) > 0) {
		normal.scale(-1);
		}
		vertex = box_one.halfSize.clone();
		if (box_one.axisVector(0).dotProduct(normal) < 0) {
		vertex.x = -vertex.x;
		}
		if (box_one.axisVector(1).dotProduct(normal) < 0) {
		vertex.y = -vertex.y;
		}
		if (box_one.axisVector(2).dotProduct(normal) < 0) {
		vertex.z = -vertex.z;
		}
		contact = new Physics.Contact(vertex.transformByMatrix(box_one.transformationMatrix), normal, penetration);
		contact.setContactData(box_two.body, box_one.body, data.restitution, data.friction);
		contacts.push(contact);
		return true;
		} else {
		best -= 6;
		box_two_axis_index = best % 3;
		box_one_axis_index = (best - box_two_axis_index) / 3;
		box_one_axis = box_one.axisVector(box_one_axis_index);
		box_two_axis = box_two.axisVector(box_two_axis_index);
		axis = box_two_axis.crossProduct(box_one_axis, new Math.Vector3D()).normalize();
		if (axis.dotProduct(to_centre) > 0) {
		axis.scale(-1);
		}
		point_on_box_one_edge = box_one.halfSize.clone();
		point_on_box_two_edge = box_two.halfSize.clone();
		for (i = 0; i <= 3; i++) {
		axis_name = (function() {
		switch (i) {
		case 0:
		return 'x';
		case 1:
		return 'y';
		case 2:
		return 'z';
		}
		})();
		if (i === box_one_axis_index) {
		point_on_box_one_edge[axis_name] = 0;
		} else if (box_one.axisVector(i).dotProduct(axis) > 0) {
		point_on_box_one_edge[axis_name] = -point_on_box_one_edge[axis_name];
		}
		if (i === box_two_axis_index) {
		point_on_box_two_edge[axis_name] = 0;
		} else if (box_two.axisVector(i).dotProduct(axis) < 0) {
		point_on_box_two_edge[axis_name] = -point_on_box_two_edge[axis_name];
		}
		}
		point_on_box_one_edge.transformByMatrix(box_one.transformationMatrix);
		point_on_box_two_edge.transformByMatrix(box_two.transformationMatrix);
		use_one = best_single_axis > 2;
		sm_one = box_one_axis.squaredMagnitude();
		sm_two = box_two_axis.squaredMagnitude();
		dp_one_two = box_two_axis.dotProduct(box_one_axis);
		to_st = point_on_box_one_edge.substract(point_on_box_two_edge, new Math.Vector3D());
		dp_st_one = box_one_axis.dotProduct(to_st);
		dp_st_two = box_two_axis.dotProduct(to_st);
		denominator = sm_one * sm_two - dp_one_two * dp_one_two;
		if (Math.abs(denominator) < 0.0001) {
		vertex = use_one ? point_on_box_one_edge : point_on_box_two_edge;
		} else {
		mua = (dp_one_two * dp_st_two - sm_two * dp_st_one) / denominator;
		mub = (sm_one * dp_st_two - dp_one_two * dp_st_one) / denominator;
		box_one_axis_name = (function() {
		switch (box_one_axis_index) {
		case 0:
		return 'x';
		case 1:
		return 'y';
		case 2:
		return 'z';
		}
		})();
		box_two_axis_name = (function() {
		switch (box_two_axis_index) {
		case 0:
		return 'x';
		case 1:
		return 'y';
		case 2:
		return 'z';
		}
		})();
		if (mua > box_one.halfSize[box_one_axis_name] || mua < -box_one.halfSize[box_one_axis_name] || mub > box_two.halfSize[box_two_axis_name] || mub < -box_two.halfSize[box_two_axis_name]) {
		vertex = use_one ? point_on_box_one_edge : point_on_box_two_edge;
		} else {
		point_on_box_one_edge.add(box_one_axis.scale(mua));
		point_on_box_two_edge.add(box_two_axis.scale(mub));
		vertex = point_on_box_one_edge.scale(0.5).add(point_on_box_two_edge.scale(0.5));
		}
		}
		contact = new Physics.Contact(vertex, axis, penetration);
		contact.setContactData(box_one.body, box_two.body, data.restitution, data.friction);
		contacts.push(contact);
		return true;
		}
		return false;
		};
		/**
		Checks for contacts between the two given bodies. This will check for contacts between all the primitives
		in the given bodies.
		@param {Physics.RigidBody} first_body The first body to check.
		@param {Physics.RigidBody} second_body The second body to check.
		@param data Data affecting the contact resolution.
		@param {Array} contacts The array where contact data will be stored.
		@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.checkForContacts = function(first_body, second_body, data, contacts) {
		var primitive_one, primitive_two, result, _i, _j, _len, _len2, _ref, _ref2;
		result = false;
		_ref = first_body.primitives;
		for (_i = 0, _len = _ref.length; _i < _len; _i++) {
		primitive_one = _ref[_i];
		_ref2 = second_body.primitives;
		for (_j = 0, _len2 = _ref2.length; _j < _len2; _j++) {
		primitive_two = _ref2[_j];
		if (primitive_one instanceof Physics.Primitives.Plane) {
		if (primitive_two instanceof Physics.Primitives.Box) {
		result = result || this.boxAndHalfSpace(primitive_two, primitive_one, data, contacts);
		} else if (primitive_two instanceof Physics.Primitives.Sphere) {
		result = result || this.sphereAndHalfSpace(primitive_two, primitive_one, data, contacts);
		}
		continue;
		}
		if (primitive_one instanceof Physics.Primitives.Box) {
		if (primitive_two instanceof Physics.Primitives.Plane) {
		result = result || this.boxAndHalfSpace(primitive_one, primitive_two, data, contacts);
		} else if (primitive_two instanceof Physics.Primitives.Sphere) {
		result = result || this.boxAndSphere(primitive_one, primitive_two, data, contacts);
		} else if (primitive_two instanceof Physics.Primitives.Box) {
		result = result || this.boxAndBox(primitive_one, primitive_two, data, contacts);
		}
		continue;
		}
		if (primitive_one instanceof Physics.Primitives.Sphere) {
		if (primitive_two instanceof Physics.Primitives.Plane) {
		result = result || this.sphereAndHalfSpace(primitive_one, primitive_two, data, contacts);
		} else if (primitive_two instanceof Physics.Primitives.Box) {
		result = result || this.boxAndSphere(primitive_two, primitive_one, data, contacts);
		} else if (primitive_two instanceof Physics.Primitives.Sphere) {
		result = result || this.sphereAndSphere(primitive_one, primitive_two, data, contacts);
		}
		continue;
		}
		}
		}
		return result;
		};
		/**
		Checks for contacts between a body and a plane. This will check for contacts between all the primitives
		in the given body.
		@param {Physics.RigidBody} body The body to check.
		@param {Physics.Primitives.Plane} plane The plane to check.
		@param data Data affecting the contact resolution.
		@param {Array} contacts The array where contact data will be stored.
		@returns {Boolean} true if contacts where found, false otherwise.
		*/
		CollisionDetector.prototype.checkForContactsWithPlane = function(body, plane, data, contacts) {
		var primitive_one, result, _i, _len, _ref;
		result = false;
		_ref = body.primitives;
		for (_i = 0, _len = _ref.length; _i < _len; _i++) {
		primitive_one = _ref[_i];
		if (primitive_one instanceof Physics.Primitives.Box) {
		result = result || this.boxAndHalfSpace(primitive_one, plane, data, contacts);
		continue;
		}
		if (primitive_one instanceof Physics.Primitives.Sphere) {
		result = result || this.sphereAndHalfSpace(primitive_one, primitive_two, data, contacts);
		continue;
		}
		}
		return result;
		};
		return CollisionDetector;
	})();
	return CollisionDetector;
});