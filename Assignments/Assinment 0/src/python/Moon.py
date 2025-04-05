class Moon:
    # Moon characteristics (from Wikipedia)
    RADIUS = 1738 * 1000        # m, Moon's Radius
    SURFACE_GRAVITY = 1.622     # m/s², Moon's Surface gravity
    ORBIT_EV = 2380             # m/s, Moon's Escape Velocity

    @staticmethod
    def gravitational_pull_acc(horizontal_speed):
        """Compute the lunar gravitational pull acceleration based on horizontal speed."""
        hs_to_es_ratio = abs(horizontal_speed) / Moon.ORBIT_EV # horizontal speed to Escape Velocity ratio 
        gp_acc = (1 - hs_to_es_ratio) * Moon.SURFACE_GRAVITY # how much the moons gravitation pulls
        return gp_acc