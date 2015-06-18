###################
path = '/home/javi/exoter/development/data/20141024_planetary_lab/20141027-2034/'
#######################################
joints_position_file = path + 'joints_position.0.data'

joints_speed_file = path + 'joints_speed.0.data'

pose_ref_position_file =  path + 'delta_pose_ref_position.0.data'

pose_odo_position_file =  path + 'delta_pose_odo_position.0.data'

pose_ref_velocity_file =  path + 'delta_pose_ref_velocity.0.data'

pose_odo_velocity_file =  path + 'delta_pose_odo_velocity.0.data'

pose_imu_orientation_file =  path + 'pose_imu_orientation.0.data'

pose_imu_angular_velocity_file =  path + 'pose_imu_angular_velocity.0.data'

pose_imu_acceleration_file =  path + 'pose_imu_acceleration.0.data'

pose_gp_odo_position_file =  path + 'delta_pose_gp_odo_position.0.data'

pose_gp_odo_velocity_file =  path + 'delta_pose_gp_odo_velocity.0.data'
#######################################

#############################
## LOAD EVALUATION TEST    ##
#############################

# Reference Robot Position
reference_position = data.ThreeData()
reference_position.readData(pose_ref_position_file, cov=True)

# Odometry Robot Position
odometry_position = data.ThreeData()
odometry_position.readData(pose_odo_position_file, cov=True)

# Reference Robot Velocity
reference_velocity = data.ThreeData()
reference_velocity.readData(pose_ref_velocity_file, cov=True)

# Odometry Robot Velocity
odometry_velocity = data.ThreeData()
odometry_velocity.readData(pose_odo_velocity_file, cov=True)

# IMU orientation
imu_orient = data.QuaternionData()
imu_orient.readData(pose_imu_orientation_file, cov=True)

# IMU acceleration
imu_acc = data.ThreeData()
imu_acc.readData(pose_imu_acceleration_file, cov=False)

# IMU Angular Velocity
imu_gyro = data.ThreeData()
imu_gyro.readData(pose_imu_angular_velocity_file, cov=False)

# Odometry Robot Position
gp_odometry_position = data.ThreeData()
gp_odometry_position.readData(pose_gp_odo_position_file, cov=True)

# Odometry Robot Velocity
gp_odometry_velocity = data.ThreeData()
gp_odometry_velocity.readData(pose_gp_odo_velocity_file, cov=True)

########################
### REMOVE OUTLIERS  ###
########################
temindex = np.where(np.isnan(reference_velocity.data[:,0]))
temindex = np.asarray(temindex)

reference_position.delete(temindex)
odometry_position.delete(temindex)
reference_velocity.delete(temindex)
odometry_velocity.delete(temindex)
imu_orient.delete(temindex)
imu_gyro.delete(temindex)
imu_acc.delete(temindex)
gp_odometry_position.delete(temindex)
gp_odometry_velocity.delete(temindex)

################################
### COMPUTE COV EIGENVALUES  ###
################################
reference_velocity.eigenValues()
odometry_velocity.eigenValues()
imu_orient.eigenValues()
imu_acc.eigenValues()
imu_gyro.eigenValues()
gp_odometry_position.eigenValues()
gp_odometry_velocity.eigenValues()

##################
# Inertia Inputs #
inertia = np.column_stack((imu_acc.getAxis(0), imu_acc.getAxis(1), imu_acc.getAxis(2),
                           imu_gyro.getAxis(0), imu_gyro.getAxis(1), imu_gyro.getAxis(2)))

######################
# Orientation Inputs #
# Roll and Pitch angles in that order #
orient = np.column_stack((imu_orient.getEuler(2), imu_orient.getEuler(1)))

reference = np.column_stack((reference_position.getAxis(0), reference_position.getAxis(1), reference_position.getAxis(2)))
#reference = np.column_stack((reference_velocity.getAxis(0), reference_velocity.getAxis(1), reference_velocity.getAxis(2)))

odometry = np.column_stack((odometry_position.getAxis(0), odometry_position.getAxis(1), odometry_position.getAxis(2)))
#odometry = np.column_stack((odometry_velocity.getAxis(0), odometry_velocity.getAxis(1), odometry_velocity.getAxis(2)))

#gp_odometry = np.column_stack((gp_odometry_position.getAxis(0), gp_odometry_position.getAxis(1), gp_odometry_position.getAxis(2)))
gp_odometry = np.column_stack((gp_odometry_velocity.getAxis(0), gp_odometry_velocity.getAxis(1), gp_odometry_velocity.getAxis(2)))


##################
## Block Filter ##
##################
sampling_frequency = 1.0/mean(reference_velocity.delta[0:100])
size_block = 1 * sampling_frequency
number_blocks = int(len(reference_velocity.delta)/size_block)

joints, jointstd = data.input_reduction(joints, number_blocks)

# Split inertia (one axis info per column)
inertia, inertiastd = data.input_reduction(inertia, number_blocks)

# Split orientation (one axis info per column)
orient, orientstd = data.input_reduction(orient, number_blocks)

# Split reference (one axis info per column)
reference, referencestd = data.input_reduction(reference, number_blocks)

# Split odometry (one axis info per column)
odometry, odometrystd = data.input_reduction(odometry, number_blocks)


# Split odometry (one axis info per column)
gp_odometry, gp_odometrystd = data.input_reduction(gp_odometry, number_blocks)


###################
matplotlib.rcParams.update({'font.size': 30, 'font.weight': 'bold'})
fig = plt.figure(3)
ax = fig.add_subplot(111)

plt.rc('text', usetex=False)# activate latex text rendering
time = odometry_velocity.time
time, timestd = data.input_reduction(time, number_blocks)
xvelocity = odometry[:,0]
ax.plot(time, xvelocity, marker='o', linestyle='-.', label="3D Odometry Velocity", color=[0.0,0.0,1.0], lw=2)

time = reference_velocity.time
time, timestd = data.input_reduction(time, number_blocks)
xvelocity = reference[:,0]
ax.scatter(time, xvelocity, marker='D', label="Reduced Reference", color=[1.0,0.0,0.0], s=80)

time = gp_odometry_velocity.time
time, timestd = data.input_reduction(time, number_blocks)
xvelocity = gp_odometry[:,0]
sigma = gp_odometry_velocity.getStd(axis=0, levelconf = 2)
sigma, sigmastd = data.input_reduction(sigma, number_blocks)
ax.plot(time, xvelocity, marker='x', linestyle='-', label="GP Odometry Velocity", color=[0.0,0.0,1.0], lw=2)
#ax.fill(np.concatenate([time, time[::-1]]), np.concatenate([xvelocity - sigma, (xvelocity + sigma)[::-1]]),
#        alpha=.5, fc='0.50', ec='None', label=r'2$\sigma$ confidence interval')
ax.fill_between(time, xvelocity - sigma, xvelocity + sigma, alpha=0.4, where=sigma <= 0.02, color='k', label='95% confidence interval')
ax.fill_between(time, xvelocity - sigma, xvelocity + sigma, alpha=0.4, where=sigma > 0.02, color='r', label='95% confidence interval')


plt.xlabel(r'Time [$s$]', fontsize=35, fontweight='bold')
plt.ylabel(r'X [$m/s$]', fontsize=35, fontweight='bold')
plt.grid(True)
ax.legend(loc=1, prop={'size':30})
plt.show(block=False)

