#include <gtest/gtest.h>

#include "bidirectional_energy_flow_control.h"

namespace variable_stiffness_controllers {
namespace {

class ConstantModelProvider : public RobotModelProvider {
 public:
  bool compute(const Vector7d& q, const Vector7d& dq,
               const ModelConfiguration& /*configuration*/,
               RobotModelState* model) const override {
    model->mass.setIdentity();
    model->coriolis.setZero();
    model->jacobian.setZero();
    model->jacobian.leftCols<6>().setIdentity();
    model->end_effector_transform.setIdentity();
    model->end_effector_transform.translation() = q.head<3>();
    model->end_effector_velocity = model->jacobian * dq;
    return true;
  }
};

BidirectionalEnergyFlowParameters makeParameters() {
  BidirectionalEnergyFlowParameters parameters;
  parameters.inertia_scale = 1.0;
  parameters.task_stiffness.diagonal() << 100.0, 100.0, 100.0, 40.0, 40.0,
      40.0;
  parameters.task_damping.diagonal() << 10.0, 10.0, 10.0, 4.0, 4.0, 4.0;
  parameters.tracking_velocity_gain.setIdentity();
  parameters.tracking_position_gain.setIdentity();
  parameters.tracking_integral_gain.setZero();
  parameters.tank_initial_energy = 1.0;
  parameters.tank_lower_bound = 0.1;
  parameters.tank_upper_bound = 1.0;
  return parameters;
}

ControlInput makeInput(const ConstantModelProvider& provider) {
  ControlInput input;
  EXPECT_TRUE(provider.compute(input.joint_position, input.joint_velocity,
                               input.model_configuration, &input.real_model));
  return input;
}

TEST(BidirectionalEnergyFlowControlTest, EquilibriumProducesZeroTorque) {
  ConstantModelProvider provider;
  BidirectionalEnergyFlowControl controller;
  const ControlInput input = makeInput(provider);
  ControlTarget target;

  ASSERT_TRUE(controller.configure(makeParameters(), &provider));
  ASSERT_TRUE(controller.initialize(input, target));
  const ControlOutput output = controller.computeControl(input, target, 0.001);

  ASSERT_TRUE(output.valid);
  EXPECT_NEAR(output.total_joint_torque.norm(), 0.0, 1.0e-12);
}

TEST(BidirectionalEnergyFlowControlTest, PositionTargetProducesFiniteTorque) {
  ConstantModelProvider provider;
  BidirectionalEnergyFlowControl controller;
  const ControlInput input = makeInput(provider);
  ControlTarget initial_target;

  ASSERT_TRUE(controller.configure(makeParameters(), &provider));
  ASSERT_TRUE(controller.initialize(input, initial_target));
  ControlTarget displaced_target = initial_target;
  displaced_target.position.x() = 0.01;
  const ControlOutput output =
      controller.computeControl(input, displaced_target, 0.001);

  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.total_joint_torque.allFinite());
  EXPECT_GT(output.total_joint_torque[0], 0.0);
}

TEST(BidirectionalEnergyFlowControlTest,
     DesiredVelocityIsNotUsedAsDampingReference) {
  ConstantModelProvider provider;
  BidirectionalEnergyFlowControl controller;
  const ControlInput input = makeInput(provider);
  ControlTarget target;
  target.velocity.setOnes();

  ASSERT_TRUE(controller.configure(makeParameters(), &provider));
  ASSERT_TRUE(controller.initialize(input, target));
  const ControlOutput output = controller.computeControl(input, target, 0.001);

  ASSERT_TRUE(output.valid);
  // Equations (5) and (14) use -D*p_dot_n and -D*p_dot respectively.
  // A desired velocity alone must therefore not generate damping torque.
  EXPECT_NEAR(output.total_joint_torque.norm(), 0.0, 1.0e-12);
}

TEST(BidirectionalEnergyFlowControlTest,
     HighTaskDampingUsesStableImplicitVelocityUpdate) {
  ConstantModelProvider provider;
  BidirectionalEnergyFlowControl controller;
  ControlInput input;
  input.joint_velocity[0] = 1.0;
  ASSERT_TRUE(provider.compute(input.joint_position, input.joint_velocity,
                               input.model_configuration, &input.real_model));

  BidirectionalEnergyFlowParameters parameters = makeParameters();
  parameters.task_stiffness.setZero();
  parameters.task_damping.setZero();
  parameters.task_damping(0, 0) = 5000.0;
  parameters.tracking_velocity_gain.setZero();
  parameters.tracking_position_gain.setZero();
  ControlTarget target;

  ASSERT_TRUE(controller.configure(parameters, &provider));
  ASSERT_TRUE(controller.initialize(input, target));
  const ControlOutput output = controller.computeControl(input, target, 0.001);

  ASSERT_TRUE(output.valid);
  // M = I, J = I and h*D = 5, hence dq_next = dq/(1 + h*D).
  EXPECT_NEAR(controller.diagnostics().nominal_joint_velocity[0], 1.0 / 6.0,
              1.0e-12);
  EXPECT_TRUE(controller.diagnostics().nominal_joint_velocity.allFinite());
}

TEST(BidirectionalEnergyFlowControlTest,
     NullspacePostureTorqueRestoresRedundantJoint) {
  ConstantModelProvider provider;
  BidirectionalEnergyFlowControl controller;
  ControlInput initial_input = makeInput(provider);
  ControlTarget target;
  BidirectionalEnergyFlowParameters parameters = makeParameters();
  parameters.tracking_velocity_gain.setZero();
  parameters.tracking_position_gain.setZero();
  parameters.nullspace_stiffness(6, 6) = 20.0;

  ASSERT_TRUE(controller.configure(parameters, &provider));
  ASSERT_TRUE(controller.initialize(initial_input, target));

  ControlInput displaced_input = initial_input;
  displaced_input.joint_position[6] = 0.1;
  ASSERT_TRUE(provider.compute(displaced_input.joint_position,
                               displaced_input.joint_velocity,
                               displaced_input.model_configuration,
                               &displaced_input.real_model));
  const ControlOutput output =
      controller.computeControl(displaced_input, target, 0.001);

  ASSERT_TRUE(output.valid);
  EXPECT_NEAR(output.total_joint_torque.head<6>().norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(output.total_joint_torque[6], -2.0, 1.0e-12);
  EXPECT_NEAR(controller.diagnostics().nullspace_torque[6], -2.0, 1.0e-12);
}

}  // namespace
}  // namespace variable_stiffness_controllers

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
