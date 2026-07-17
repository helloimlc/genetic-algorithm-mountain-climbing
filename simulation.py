import pybullet as p
from multiprocessing import Pool
import sys, os
sys.path.append(os.path.dirname(__file__))
import time 

from cw_envt import load_terrain
from cw_envt import load_terrain_hill_and_valleys


class Simulation: 
    def __init__(self, sim_id=0, gui=False):
        self.sim_id = sim_id
        self.physicsClientId = p.connect(p.GUI if gui else p.DIRECT)

        # Load terrain, choose mountain or hill and valley, uncomment to change
        load_terrain()
        # load_terrain_hill_and_valleys()

        self.set_camera_view(distance=10, yaw=50, pitch=-30, target_pos=[5,5,0])
    
    def set_camera_view(self, distance, yaw, pitch, target_pos):
        p.resetDebugVisualizerCamera(cameraDistance=distance,
                                     cameraYaw=yaw,
                                     cameraPitch=pitch,
                                     cameraTargetPosition=target_pos,
                                     physicsClientId=self.physicsClientId)
        
    def run_creature(self, cr, iterations=2400):
        pid = self.physicsClientId
        #p.resetSimulation(physicsClientId=pid)
        p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=pid)
        p.setGravity(0, 0, -10, physicsClientId=pid)

        # Load creature
        xml_file = f'temp{cr.id}.urdf'
        #xml_file = f'temp{self.sim_id}.urdf'
        with open(xml_file, 'w') as f:
            f.write(cr.to_xml())

        # Spawn the creature near the bottom-left of the mountain
        spawn_x = 4.5   # toward the start of the slope
        spawn_y = 4.5    # centered in y
        spawn_z = 2    # slightly above ground to avoid collision

        cid = p.loadURDF(xml_file, [spawn_x, spawn_y, spawn_z], [0, 0, 0, 1], physicsClientId=pid)

        try:
            ground_id = 0  # Usually ID 0 if the ground is the first loaded object
            cr.start_position = self.wait_for_ground_contact(cid, ground_id, pid)

            #cr.start_position = p.getBasePositionAndOrientation(cid, physicsClientId=pid)[0]
        except Exception as e:
            print({e})

        for step in range(iterations):
            p.stepSimulation(physicsClientId=pid)
            if step % 24 == 0:
                self.update_motors(cid, cr)
            if self.physicsClientId == p.GUI:
                time.sleep(10.0 / 240.0)  # Slow down GUI
            if step == iterations - 1:
                #pos, _ = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
                try:
                    pos, _ = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
                except Exception as e:
                    print({e})
                cr.update_position([0, 0, 0])  # or skip update / set to a safe default
        os.remove(xml_file)
        p.removeBody(cid, physicsClientId=pid)

    def update_motors(self, cid, cr):
        for jid in range(p.getNumJoints(cid, physicsClientId=self.physicsClientId)):
            m = cr.get_motors()[jid]
            p.setJointMotorControl2(cid, jid, 
                controlMode=p.VELOCITY_CONTROL, 
                targetVelocity=m.get_output(), 
                force=5, 
                physicsClientId=self.physicsClientId)

    def eval_population(self, pop, iterations):
        for cr in pop.creatures:
            self.run_creature(cr, iterations)
        #print("")

    def wait_for_ground_contact(self, cid, ground_id, pid, timeout=240):
        for _ in range(timeout):
            p.stepSimulation(physicsClientId=pid)
            contacts = p.getContactPoints(bodyA=cid, bodyB=ground_id, physicsClientId=pid)
            if contacts:
                pos, _ = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
                return pos
        # fallback if never touches
        return p.getBasePositionAndOrientation(cid, physicsClientId=pid)[0]


class ThreadedSim():
    def __init__(self, pool_size):
        self.sims = [Simulation(i) for i in range(pool_size)]

    @staticmethod
    def static_run_creature(sim, cr, iterations):
        sim.run_creature(cr, iterations)
        return cr
    
    def eval_population(self, pop, iterations):
        pool_args = [] 
        start_ind = 0
        pool_size = len(self.sims)
        while start_ind < len(pop.creatures):
            this_pool_args = []
            for i in range(start_ind, start_ind + pool_size):
                if i == len(pop.creatures): break
                sim_ind = i % len(self.sims)
                this_pool_args.append([
                    self.sims[sim_ind], 
                    pop.creatures[i], 
                    iterations
                ])
            pool_args.append(this_pool_args)
            start_ind += pool_size

        new_creatures = []
        for pool_argset in pool_args:
            with Pool(pool_size) as p:
                creatures = p.starmap(ThreadedSim.static_run_creature, pool_argset)
                new_creatures.extend(creatures)
        pop.creatures = new_creatures
