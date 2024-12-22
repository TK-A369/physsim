pub mod physics;

pub use physics::*;

#[cfg(test)]
mod tests {
    use super::*;

    use dashu;

    #[test]
    fn rigid_body_test_1_f32() {
        let mut rb = RigidBody::<f32> {
            pos: nalgebra::Vector3::new(0.0, 0.0, 0.0),
            lin_vel: nalgebra::Vector3::new(0.1, 0.2, 0.0),
            rot_mat: nalgebra::Matrix3::identity(),
            ang_mom: nalgebra::Vector3::new(0.0, -0.3, 0.7),
            inv_ine: nalgebra::Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        };
        for _ in 0..4 {
            rb.step_sim(1.0);
            println!("{:?}", rb);
        }
    }

    #[test]
    fn rigid_body_test_1_f64() {
        let mut rb = RigidBody::<f64> {
            pos: nalgebra::Vector3::new(0.0, 0.0, 0.0),
            lin_vel: nalgebra::Vector3::new(0.1, 0.2, 0.0),
            rot_mat: nalgebra::Matrix3::identity(),
            ang_mom: nalgebra::Vector3::new(0.0, -0.3, 0.7),
            inv_ine: nalgebra::Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        };
        for _ in 0..4 {
            rb.step_sim(1.0);
            println!("{:?}", rb);
        }
    }

    #[test]
    fn rigid_body_test_1_dashu() {
        let mut rb = RigidBody::<dashu::Real> {
            pos: nalgebra::Vector3::new(
                dashu::Real::try_from(0.0).unwrap(),
                dashu::Real::try_from(0.0).unwrap(),
                dashu::Real::try_from(0.0).unwrap(),
            ),
            lin_vel: nalgebra::Vector3::new(
                dashu::Real::try_from(0.1).unwrap(),
                dashu::Real::try_from(0.2).unwrap(),
                dashu::Real::try_from(0.0).unwrap(),
            ),
            rot_mat: nalgebra::Matrix3::identity(),
            ang_mom: nalgebra::Vector3::new(
                dashu::Real::try_from(0.0).unwrap(),
                dashu::Real::try_from(-0.3).unwrap(),
                dashu::Real::try_from(0.7).unwrap(),
            ),
            inv_ine: nalgebra::Matrix3::new(
                dashu::Real::try_from(1.0).unwrap(),
                dashu::Real::try_from(0.0).unwrap(),
                dashu::Real::try_from(0.0).unwrap(),
                dashu::Real::try_from(0.0).unwrap(),
                dashu::Real::try_from(1.0).unwrap(),
                dashu::Real::try_from(0.0).unwrap(),
                dashu::Real::try_from(0.0).unwrap(),
                dashu::Real::try_from(0.0).unwrap(),
                dashu::Real::try_from(1.0).unwrap(),
            ),
        };
        for _ in 0..4 {
            rb.step_sim(dashu::Real::try_from(1.0).unwrap());
            println!("{:?}", rb);
        }
    }
}
