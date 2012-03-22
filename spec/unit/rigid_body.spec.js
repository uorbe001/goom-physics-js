var requirejs = require("requirejs");
requirejs.config({nodeRequire: require});

requirejs(["../../src/rigid_body", "goom-math"], function(RigidBody, Mathematics) {
	describe("RigidBody", function() {
		beforeEach(function() {
			this.body = new RigidBody();
		});

		it("should set the inverse inertia tensor when setInertiaTensor is called", function(){
			var inert = new Mathematics.Matrix3D();
			inert.setDiagonal(3, 3, 3);
			this.body.setInertiaTensor(inert);
			inert.inverse();
			for (var i = 0; i <= 8; i++) {
				expect(this.body.inverseInertiaTensor.data[i]).toBe(inert.data[i]);
			}
		});

		it("should set the inverse inertia tensor when setInertiaTensor is called", function(){
			var inert = new Mathematics.Matrix3D();
			inert.setDiagonal(3, 3, 3).inverse();
			this.body.setInertiaTensorCoefficients(3, 3, 3);
			for (var i = 0; i <= 8; i++) {
				expect(this.body.inverseInertiaTensor.data[i]).toBe(inert.data[i]);
			}
		});

		it("should call all the functions used to update internal data when calculateInternalData is used", function() {
			this.body.calculateInternalData();
			var mat = new Mathematics.Matrix4D();
			mat.makeFromPositionAndOrientation(this.body.position, this.body.orientation);
			for (var i = 0; i <= 15; i++) {
				expect(this.body.transformationMatrix.data[i]).toBe(mat.data[i]);
			}

			mat = new Mathematics.Matrix3D();
			this.body.transformationMatrix.transformMatrix3D(this.body.inverseInertiaTensor, mat);
			for (i = 0; i <= 8; i++) {
				expect(this.body.inverseInertiaTensorWorld.data[i]).toBe(mat.data[i]);
			}
			//TODO: BoundingVolumeHierarchyNode is updated?
			//TODO: What about the primitives, are they being updated?
		});

		it("should increase the motion when waking up the body", function() {
			this.body.isAwake = false;
			this.body.wakeUp();
			expect(this.body.isAwake).toBeTruthy();
			expect(this.body.motion).toBeGreaterThan(RigidBody.SLEEP_EPSILON);
		});

		it("should clear velocities when the body goes to sleep", function() {
			this.body.velocity.set(1, 2, 3);
			this.body.angular_velocity.set(3, 2, 1);
			this.body.sleep();
			expect(this.body.isAwake).toBeFalsy();
			expect(this.body.velocity.x).toBe(0);
			expect(this.body.velocity.y).toBe(0);
			expect(this.body.velocity.z).toBe(0);
			expect(this.body.angular_velocity.x).toBe(0);
			expect(this.body.angular_velocity.y).toBe(0);
			expect(this.body.angular_velocity.z).toBe(0);
		});

		it("should add the force to the accumulated force and wake the body up when applyForce is called", function() {
			var force = new Mathematics.Vector3D(1, 2, 3);
			this.body.wakeUp = jasmine.createSpy();
			expect(this.body.accumulatedForce.x).toBe(0);
			expect(this.body.accumulatedForce.y).toBe(0);
			expect(this.body.accumulatedForce.z).toBe(0);
			this.body.applyForce(force);
			expect(this.body.accumulatedForce.x).toBe(1);
			expect(this.body.accumulatedForce.y).toBe(2);
			expect(this.body.accumulatedForce.z).toBe(3);
			expect(this.body.wakeUp).toHaveBeenCalled();
			this.body.applyForce(force);
			expect(this.body.accumulatedForce.x).toBe(2);
			expect(this.body.accumulatedForce.y).toBe(4);
			expect(this.body.accumulatedForce.z).toBe(6);
			expect(this.body.wakeUp).toHaveBeenCalled();
		});

		it("should update force to the accumulatedForce and accumulatedTorque and wake the body up when the force is applied at a point", function() {
			var force = new Mathematics.Vector3D(0, 1, 0);
			var point = new Mathematics.Vector3D(1, -1, 1);
			this.body.wakeUp = jasmine.createSpy();
			this.body.applyForceAtPoint(force, point);
			expect(this.body.accumulatedForce.x).toBe(0);
			expect(this.body.accumulatedForce.y).toBe(1);
			expect(this.body.accumulatedForce.z).toBe(0);
			expect(this.body.accumulatedTorque.x).toBe(-1);
			expect(this.body.accumulatedTorque.y).toBe(0);
			expect(this.body.accumulatedTorque.z).toBe(1);
			expect(this.body.wakeUp).toHaveBeenCalled();
			this.body.applyForceAtPoint(force, point);
			expect(this.body.accumulatedForce.x).toBe(0);
			expect(this.body.accumulatedForce.y).toBe(2);
			expect(this.body.accumulatedForce.z).toBe(0);
			expect(this.body.accumulatedTorque.x).toBe(-2);
			expect(this.body.accumulatedTorque.y).toBe(0);
			expect(this.body.accumulatedTorque.z).toBe(2);
			expect(this.body.wakeUp).toHaveBeenCalled();
			//TODO: Not sure if this is right.
			//TODO: Not sure if this is right.
			//TODO: Test different forces and positions!
		});

		it("should update force to the accumulatedForce and accumulatedTorque and wake the body up when the force is applied at a body point", function() {
			var force = new Mathematics.Vector3D(0, 1, 0);
			var point = new Mathematics.Vector3D(1, -1, 1);
			this.body.wakeUp = jasmine.createSpy();
			this.body.applyForceAtPoint(force, point);
			expect(this.body.accumulatedForce.x).toBe(0);
			expect(this.body.accumulatedForce.y).toBe(1);
			expect(this.body.accumulatedForce.z).toBe(0);
			expect(this.body.accumulatedTorque.x).toBe(-1);
			expect(this.body.accumulatedTorque.y).toBe(0);
			expect(this.body.accumulatedTorque.z).toBe(1);
			expect(this.body.wakeUp).toHaveBeenCalled();
			this.body.applyForceAtPoint(force, point);
			expect(this.body.accumulatedForce.x).toBe(0);
			expect(this.body.accumulatedForce.y).toBe(2);
			expect(this.body.accumulatedForce.z).toBe(0);
			expect(this.body.accumulatedTorque.x).toBe(-2);
			expect(this.body.accumulatedTorque.y).toBe(0);
			expect(this.body.accumulatedTorque.z).toBe(2);
			expect(this.body.wakeUp).toHaveBeenCalled();
			//TODO: Not sure if this is right.
			//TODO: Test different forces and positions!
		});

		it("should add the torque to the accumulated torque and wake the body up when applyTorque is called", function() {
			var force = new Mathematics.Vector3D(1, 2, 3);
			this.body.wakeUp = jasmine.createSpy();
			expect(this.body.accumulatedTorque.x).toBe(0);
			expect(this.body.accumulatedTorque.y).toBe(0);
			expect(this.body.accumulatedTorque.z).toBe(0);
			this.body.applyTorque(force);
			expect(this.body.accumulatedTorque.x).toBe(1);
			expect(this.body.accumulatedTorque.y).toBe(2);
			expect(this.body.accumulatedTorque.z).toBe(3);
			expect(this.body.wakeUp).toHaveBeenCalled();
			this.body.applyTorque(force);
			expect(this.body.accumulatedTorque.x).toBe(2);
			expect(this.body.accumulatedTorque.y).toBe(4);
			expect(this.body.accumulatedTorque.z).toBe(6);
			expect(this.body.wakeUp).toHaveBeenCalled();
		});

		it("should add the torque to the accumulated torque and wake the body up when applyTorque is called", function() {
			this.body.accumulatedForce.set(1, 2, 3);
			this.body.accumulatedTorque.set(1, 2, 3);
			this.body.clear();
			expect(this.body.accumulatedTorque.x).toBe(0);
			expect(this.body.accumulatedTorque.y).toBe(0);
			expect(this.body.accumulatedTorque.z).toBe(0);
			expect(this.body.accumulatedForce.x).toBe(0);
			expect(this.body.accumulatedForce.y).toBe(0);
			expect(this.body.accumulatedForce.z).toBe(0);
		});

		it("should tell whether the body has finite mass", function() {
			this.body.setMass(-1);
			expect(this.body.hasFiniteMass()).toBeFalsy();
			this.body.setMass(10);
			expect(this.body.hasFiniteMass()).toBeTruthy();
			this.body.setMass(0);
			expect(this.body.hasFiniteMass()).toBeTruthy();
		});

		it("should get and set the object's mass", function() {
			this.body.setMass(0);
			expect(this.body.getMass()).toBe(0);
			this.body.setMass(10);
			expect(this.body.getMass()).toBe(10);
		});

		it("should not update anything when the body is asleep", function(){
			this.body.sleep();
			this.body.setMass(1);
			this.body.velocity.set(1, 1, 1);
			this.body.angular_velocity.set(1, 1, 1);
			this.body.integrate(1);
			expect(this.body.position.x).toBe(0);
			expect(this.body.position.y).toBe(0);
			expect(this.body.position.z).toBe(0);
			expect(this.body.orientation.r).toBe(1);
			expect(this.body.orientation.i).toBe(0);
			expect(this.body.orientation.j).toBe(0);
			expect(this.body.orientation.k).toBe(0);
		});

		it("should update the position and orientation when the body is awake", function(){
			var q = new Mathematics.Quaternion();
			q.addVector(new Mathematics.Vector3D(1 * RigidBody.ANGULAR_DAMPING, 1 * RigidBody.ANGULAR_DAMPING, 1 * RigidBody.ANGULAR_DAMPING)).normalize();
			this.body.angular_velocity.set(1, 1, 1);
			this.body.setMass(1);
			this.body.accumulatedForce.set(1, 1, 1);
			this.body.integrate(1);
			expect(Math.round(this.body.position.x)).toBe(Math.round(1*RigidBody.LINEAR_DAMPING));
			expect(Math.round(this.body.position.y)).toBe(Math.round(1*RigidBody.LINEAR_DAMPING));
			expect(Math.round(this.body.position.z)).toBe(Math.round(1*RigidBody.LINEAR_DAMPING));
			expect(this.body.orientation.r).toBe(q.r);
			expect(this.body.orientation.i).toBe(q.i);
			expect(this.body.orientation.j).toBe(q.j);
			expect(this.body.orientation.k).toBe(q.k);
			expect(this.body.lastFrameAcceleration.x).toBe(1);
			expect(this.body.lastFrameAcceleration.y).toBe(1);
			expect(this.body.lastFrameAcceleration.z).toBe(1);
			expect(this.body.acceleration.x).toBe(0);
			expect(this.body.acceleration.y).toBe(0);
			expect(this.body.acceleration.z).toBe(0);
			this.body.integrate(1);
			q.addVector(new Mathematics.Vector3D(1 * RigidBody.ANGULAR_DAMPING, 1 * RigidBody.ANGULAR_DAMPING, 1 * RigidBody.ANGULAR_DAMPING)).normalize();
			expect(Math.round(this.body.position.x)).toBe(Math.round(2 * RigidBody.LINEAR_DAMPING));
			expect(Math.round(this.body.position.y)).toBe(Math.round(2 * RigidBody.LINEAR_DAMPING));
			expect(Math.round(this.body.position.z)).toBe(Math.round(2 * RigidBody.LINEAR_DAMPING));
			expect(Math.round(this.body.orientation.r)).toBe(Math.round(q.r));
			expect(Math.round(this.body.orientation.i)).toBe(Math.round(q.i));
			expect(Math.round(this.body.orientation.j)).toBe(Math.round(q.j));
			expect(Math.round(this.body.orientation.k)).toBe(Math.round(q.k));
			expect(this.body.lastFrameAcceleration.x).toBe(0);
			expect(this.body.lastFrameAcceleration.y).toBe(0);
			expect(this.body.lastFrameAcceleration.z).toBe(0);
			expect(this.body.acceleration.x).toBe(0);
			expect(this.body.acceleration.y).toBe(0);
			expect(this.body.acceleration.z).toBe(0);
		});

		//TODO: Activate when the primitives are added.
		xit("should add proper primitives to the body when addPrimitives is called", function() {
			this.body.addPrimitives({type: "sphere", radious: 2, offset: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]});
			expect(this.body.primitives.length).toBe(1);
			expect(this.body.primitives[0] instanceof Physics.Primitives.Sphere).toBeTruthy();
			expect(this.body.primitives[0].body).toBe(this.body);
			
			var body2 = new Physics.RigidBody();
			body2.addPrimitives([{type: "sphere", radious: 2, offset: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]}, {type: "box", halfSize: {x: 10, y: 5, z: 5}, offset: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1]}]);
			expect(body2.primitives.length).toBe(2);
			expect(body2.primitives[0] instanceof Physics.Primitives.Sphere).toBeTruthy();
			expect(body2.primitives[1] instanceof Physics.Primitives.Box).toBeTruthy();
			expect(body2.primitives[0].body).toBe(body2);
			expect(body2.primitives[1].body).toBe(body2);

			this.body = new Physics.RigidBody();
			this.body.addPrimitives({type: "sphere", radious: 2});
			expect(this.body.primitives.length).toBe(1);
			expect(this.body.primitives[0] instanceof Physics.Primitives.Sphere).toBeTruthy();
			expect(this.body.primitives[0].body).toBe(this.body);

			body2 = new Physics.RigidBody();
			body2.addPrimitives([{type: "sphere", radious: 2}, {type: "box", halfSize: {x: 10,y: 5,z: 5}}]);
			expect(body2.primitives.length).toBe(2);
			expect(body2.primitives[0] instanceof Physics.Primitives.Sphere).toBeTruthy();
			expect(body2.primitives[1] instanceof Physics.Primitives.Box).toBeTruthy();
			expect(body2.primitives[0].body).toBe(body2);
			expect(body2.primitives[1].body).toBe(body2);
		});
	});
});