#FrontEnd reference from Ground Truth
frontendreference100Hz = ThreeData()
#frontendreference100Hz.readData('data/normal_spacehall/frontend_referencepose_velocity.1154.0.data', cov=True)
#frontendreference100Hz.readData('data/normal_spacehall/frontend_referencepose_position.1154.0.data', cov=True)
frontendreference100Hz.readData('data/normal_spacehall/frontend_referencepose_velocity.1112.0.data', cov=True)
frontendreference100Hz.eigenValues()

#FrontEnd Motion model velocity
frontendbody100Hz = ThreeData()
#frontendbody100Hz.readData('data/normal_spacehall/frontend_poseout_velocity.1154.1.data', cov=True)
#frontendbody100Hz.readData('data/normal_spacehall/frontend_poseout_position.1154.1.data', cov=True)
frontendbody100Hz.readData('data/normal_spacehall/frontend_poseout_velocity.1112.0.data', cov=True)
frontendbody100Hz.eigenValues()

#Back End Pose Out velocity
backendvelo100Hz = ThreeData()
#backendvelo100Hz.readData('data/normal_spacehall/backend_poseout_velocity.1154.1.data', cov=True)
#backendvelo100Hz.readData('data/normal_spacehall/backend_poseout_position.1154.2.data', cov=True)
backendvelo100Hz.readData('data/normal_spacehall/backend_poseout_velocity.1112.0.data', cov=True)
backendvelo100Hz.eigenValues()


#Back End Front End velocity comparisons
plt.figure(1)
values = frontendbody100Hz.getAxis(0)
plt.plot(frontendbody100Hz.time, values,
        marker='.', label="Motion Model X-axis", color=[1,0,1], lw=2)
plt.plot(frontendbody100Hz.time, frontendbody100Hz.getStdMax(0, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(frontendbody100Hz.time, frontendbody100Hz.getStdMin(0, 3) , color=[0,0,0], linestyle='--', lw=2)
values = backendvelo100Hz.getAxis(0)
plt.plot(backendvelo100Hz.time, values,
        marker='.', label="Filter X-axis", color=[1,0.8,0], lw=2)
plt.plot(backendvelo100Hz.time, backendvelo100Hz.getStdMax(0, 3) , color=[0.4,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(backendvelo100Hz.time, backendvelo100Hz.getStdMin(0, 3) , color=[0.4,0,0], linestyle='--', lw=2)

values=frontendreference100Hz.getAxis(0)
plt.plot(frontendreference100Hz.time, values,
        marker='D', label="Ground Truth X-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/motion_model_vs_filter_predict_velocity_x_velocity_plot.png')

plt.figure(1)
values = frontendbody100Hz.getAxis(1)
plt.plot(frontendbody100Hz.time, values,
        marker='.', label="Motion Model Y-axis", color=[1,0,0], lw=2)
plt.plot(frontendbody100Hz.time, frontendbody100Hz.getStdMax(1, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(frontendbody100Hz.time, frontendbody100Hz.getStdMin(1, 3) , color=[0,0,0], linestyle='--', lw=2)
values = backendvelo100Hz.getAxis(1)
plt.plot(backendvelo100Hz.time, values,
        marker='.', label="Filter Y-axis", color=[1,0.6,0], lw=2)
plt.plot(backendvelo100Hz.time, backendvelo100Hz.getStdMax(1, 3) , color=[0,0.4,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(backendvelo100Hz.time, backendvelo100Hz.getStdMin(1, 3) , color=[0,0.4,0], linestyle='--', lw=2)

values=frontendreference100Hz.getAxis(1)
plt.plot(frontendreference100Hz.time, values,
        marker='D', label="Ground Truth Y-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/motion_model_vs_filter_predict_velocity_y_velocity_plot.png')


plt.figure(2)
values = frontendbody100Hz.getAxis(2)
plt.plot(frontendbody100Hz.time, values,
        marker='.', label="Motion Model Z-axis", color=[1,0,0], lw=2)
plt.plot(frontendbody100Hz.time, frontendbody100Hz.getStdMax(2, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(frontendbody100Hz.time, frontendbody100Hz.getStdMin(2, 3) , color=[0,0,0], linestyle='--', lw=2)
values = backendvelo100Hz.getAxis(2)
plt.plot(backendvelo100Hz.time, values,
        marker='.', label="Filter Z-axis", color=[1,0.6,0], lw=2)
plt.plot(backendvelo100Hz.time, backendvelo100Hz.getStdMax(2, 3) , color=[0,0,0.8], linestyle='--', lw=2, label=r'$\pm 3\sigma$ uncertainty')
plt.plot(backendvelo100Hz.time, backendvelo100Hz.getStdMin(2, 3) , color=[0,0,0.8], linestyle='--', lw=2)

values=frontendreference100Hz.getAxis(2)
plt.plot(frontendreference100Hz.time, values,
        marker='D', label="Ground Truth Z-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/motion_model_vs_filter_predict_velocity_z_velocity_plot.png')


#FrontEnd reference from Ground Truth
frontendreference100Hz = ThreeData()
frontendreference100Hz.readData('data/normal_spacehall/frontend_referencepose_velocity.1154.0.data', cov=True)
#frontendreference100Hz.readData('data/multitest_spacehall/frontend_referencepose_velocity.0940.1.data', cov=True)
frontendreference100Hz.eigenValues()

#FrontEnd Motion model velocity
frontendbody100Hz = ThreeData()
frontendbody100Hz.readData('data/normal_spacehall/frontend_poseout_velocity.1154.0.data', cov=True)
#frontendbody100Hz.readData('data/multitest_spacehall/frontend_poseout_velocity.0940.1.data', cov=True)
frontendbody100Hz.eigenValues()

#DeltaVelocity from MotionModel
backenddeltamodel10Hz = ThreeData()
backenddeltamodel10Hz.readData('data/normal_spacehall/backend_delta_velo_model.1154.1.data', cov=True)
backenddeltamodel10Hz.eigenValues()

#Acc from MotionModel
backendaccmodel10Hz = ThreeData()
backendaccmodel10Hz.readData('data/normal_spacehall/backend_acc_model.1154.1.data', cov=True)
backendaccmodel10Hz.eigenValues()

#DeltaVelocity from InertialState
backenddeltainertial10Hz = ThreeData()
backenddeltainertial10Hz.readData('data/normal_spacehall/backend_delta_velo_inertial.1154.1.data', cov=True)
backenddeltainertial10Hz.eigenValues()

#Acc from InertialState
backendaccinertial10Hz = ThreeData()
backendaccinertial10Hz.readData('data/normal_spacehall/backend_acc_inertial.1154.1.data', cov=True)
backendaccinertial10Hz.eigenValues()

#DeltaVelocity Error
backenddeltaerror10Hz = ThreeData()
backenddeltaerror10Hz.readData('data/normal_spacehall/backend_delta_velo_error.1154.0.data', cov=True)
backenddeltaerror10Hz.eigenValues()

#DeltaVelocity Common
backenddeltacommon10Hz = ThreeData()
backenddeltacommon10Hz.readData('data/normal_spacehall/backend_delta_velo_common.1154.0.data', cov=True)
backenddeltacommon10Hz.eigenValues()

#Hellinger Coeff
backendhellinger10Hz = ThreeData()
backendhellinger10Hz.readData('data/normal_spacehall/backend_hellinger.1154.0.data', cov=False)
backendhellinger10Hz.eigenValues()

#Bhattacharyya Coeff
backendbhatta10Hz = ThreeData()
backendbhatta10Hz.readData('data/normal_spacehall/backend_bhatta.1154.0.data', cov=False)
backendbhatta10Hz.eigenValues()

#Threshold Coeff
backendthreshold10Hz = ThreeData()
backendthreshold10Hz.readData('data/normal_spacehall/backend_threshold.1154.0.data', cov=False)
backendthreshold10Hz.eigenValues()

#Mahalanobis distance
backendmahalanobis10Hz = OneData()
backendmahalanobis10Hz.readData('data/normal_spacehall/backend_mahalanobis.1154.0.data', cov=False)

#Sensitivity analysis
sensitivitytstate = OneData()
sensitivitytstate.readData('data/normal_spacehall/frontend_sensitivity_tstate.1154.0.data', cov=False)

#Sensitivity analysis
sensitivitytcov = OneData()
sensitivitytcov.readData('data/normal_spacehall/frontend_sensitivity_tcovariance_RL.1154.0.data', cov=False)


frontendbody100Hz.plot_axis2(1, 0, True, 1, True, [0,1,1])
frontendbody100Hz.plot_errorbars(1, 0,True, 1, True, [0,1,1])
frontendbody100Hz.plot_axis(1, 1,False, 1, True, [0,1,0])
frontendbody100Hz.plot_axis(1, 2,False, 1, True, [1,0,0])

frontendreference100Hz.plot_axis(1, 0,False, 1, True, [0.5,1,0])
frontendreference100Hz.plot_axis(1, 1,False, 1, True, [1,0,0])

backenddeltamodel10Hz.plot_axis(1, 0, True, 1, True, [0,0,1])
backenddeltainertial10Hz.plot_axis(1, 0, True, 1, True, [0,1,0])
backenddeltacommon10Hz.plot_axis(1, 0, True, 1, True, [0.5,0.5,0.5])
backenddeltaerror10Hz.plot_axis(1, 0, True, 1, True, [1,0,0])
backendhellinger10Hz.plot_axis(1, 0, False, 1, True, [0,0,0])
backendbhatta10Hz.plot_axis(1, 0, False, 1, True, [0,0,0])
backendmahalanobis10Hz.plot_axis(1, 0, False, 1, True, [1,0,0])
backendthreshold10Hz.plot_axis(1, 0, False, 1, True, [0,0,0])

backenddeltamodel10Hz.plot_axis(2, 1, True, 1, True, [0,0,1])
backenddeltainertial10Hz.plot_axis(2, 1, True, 1, True, [0,1,0])
backenddeltaerror10Hz.plot_axis(2, 1, True, 1, True, [1,0,0])

backenddeltamodel10Hz.plot_axis(3, 2, True, 1, True, [0,0,1])
backenddeltainertial10Hz.plot_axis(3, 2, True, 1, True, [0,1,0])

backendaccmodel10Hz.plot_axis(2, 0, False, 1, True, [0,0,1])
backendaccinertial10Hz.plot_axis(2, 0, False, 1, True, [0,1,0])

velocommon = []
velocommon.append(backenddeltacommon10Hz.data[0][0])
for i in range(1,len(backenddeltacommon10Hz.t)):
    velocommon.append(velocommon[i-1] + backenddeltacommon10Hz.data[i][0])

plot(backenddeltacommon10Hz.t,velocommon, '-o', label="X common velocity", color='green')

veloinertial = []
veloinertial.append(backenddeltainertial10Hz.data[0][0])
for i in range(1,len(backenddeltainertial10Hz.t)):
    veloinertial.append(veloinertial[i-1] + backenddeltainertial10Hz.data[i][0])

plot(backenddeltainertial10Hz.t,veloinertial, '-o', label="X common velocity", color='red')


#Customized plotting
plt.figure(1)
errorbar(frontendbody100Hz.t, frontendbody100Hz.getAxis(0),
           yerr=frontendbody100Hz.getStd(0),
           marker='D',
           color='k',
           ecolor='r',
           alpha=0.5,
           markerfacecolor='b',
           label="",
           capsize=0,
           linestyle='--')
plt.grid(True)
plt.show(block=False)

plt.figure(1)
values = frontendbody100Hz.getAxis(0)
plt.plot(frontendbody100Hz.t, values,
        marker='.', label="Motion Model X-axis", color=[1,0,0], lw=2)
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMax(0, 3) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 1\sigma$ uncertainty')
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMin(0, 3) , color=[0,0,0], linestyle='--', lw=2)
values=frontendreference100Hz.getAxis(0)
plt.plot(frontendreference100Hz.t, values,
        marker='D', label="Ground Truth X-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/turn_point_x_velocity_plot.png')

plt.figure(1)
values = frontendbody100Hz.getAxis(1)
plt.plot(frontendbody100Hz.t, values,
        marker='.', label="Motion Model Y-axis", color=[0,1,0], lw=2)
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMax(1) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 1\sigma$ uncertainty')
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMin(1) , color=[0,0,0], linestyle='--', lw=2)
values=frontendreference100Hz.getAxis(1)
plt.plot(frontendreference100Hz.t, values,
        marker='D', label="Ground Truth Y-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/turn_point_y_velocity_plot.png')

plt.figure(1)
values = frontendbody100Hz.getAxis(2)
plt.plot(frontendbody100Hz.t, values,
        marker='.', label="Motion Model Z-axis", color=[0,0,1], lw=2)
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMax(2) , color=[0,0,0], linestyle='--', lw=2, label=r'$\pm 1\sigma$ uncertainty')
plt.plot(frontendbody100Hz.t, frontendbody100Hz.getStdMin(2) , color=[0,0,0], linestyle='--', lw=2)
values=frontendreference100Hz.getAxis(2)
plt.plot(frontendreference100Hz.t, values,
        marker='D', label="Ground Truth Z-axis", color=[0,0.5,0.5], alpha=0.5, lw=5)
plt.ylabel(r'Velocity [$m/s$]')
plt.xlabel(r'Time [$s$]')
plt.grid(True)
plt.legend(prop={'size':25})
plt.show(block=False)
savefig('figures/turn_point_z_velocity_plot.png')


