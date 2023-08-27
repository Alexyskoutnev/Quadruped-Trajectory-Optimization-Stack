class State_Estimation:
    
    def __init__(self, init_pos=(0, 0, 0), init_vel=(0, 0, 0)):
        self.CoM = init_pos
        self.CoM_vel = init_vel

    def RK4(self, imu_data, timestep, val):
        """Integrator scheme for imu data

        Args:
            imu_data (_type_): _description_
            timestep (_type_): _description_
            val (_type_): _description_

        Returns:
            _type_: _description_
        """
        k1_vx = imu_data[0] * timestep
        k1_vy = imu_data[1] * timestep

        k2_vx = (imu_data[0] + k1_vx / 2) * timestep
        k2_vy = (imu_data[1] + k1_vx / 2) * timestep

        k3_vx = (imu_data[0] + k2_vx / 2) * timestep
        k3_vy = (imu_data[1] + k2_vy / 2) * timestep

        k4_vx = (imu_data[0] + k3_vx) * timestep
        k4_vy = (imu_data[1] + k3_vy) * timestep
        
        val[0] += (k1_vx + 2 * k2_vx + 2 * k3_vx + k4_vx) / 6.0
        val[0] += (k1_vy + 2 * k2_vy + 2 * k3_vy + k4_vy) / 6.0

        return val

    def update(self, imu_data, timestep):
        self.CoM_vel = self.RK4(imu_data, timestep, self.CoM_vel)
        self.CoM = (self.CoM[0] + self.CoM_vel[0] * timestep, self.CoM[1] + self.CoM_vel[1] * timestep)



        




