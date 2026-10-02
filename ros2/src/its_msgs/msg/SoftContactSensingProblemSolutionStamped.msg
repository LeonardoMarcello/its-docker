# All values are referenced in the fingertip core center frame {B}

std_msgs/Header header              # stamp
its_msgs/ContactSensingProblemSolution csp                    # Point of Contact [mm]
# Definition of ContactSensingProblemSolution.msg:
#   geometry_msgs/Point c               # Contact Centroid [mm]
#   geometry_msgs/Vector3 n             # normal direction to fingertip surfaces in PoC
#   float64 fn                          # amplitude [N] of normal force
#   geometry_msgs/Vector3 ft            # tangential force [N]
#   float64 t                           # amplitude [Nmm] of local torque around normal
#
#
#   float64 convergence_time            # convergence time [ms]


float64 d                           # deformation [mm] of fingertip surfaces along normal
