//use crate::math;

use std::ops;

use nalgebra;
use num_traits;

#[derive(Debug)]
pub struct RigidBody<NumType> {
    //pub pos: crate::math::Vec3<NumType>,
    //pub lin_vel: crate::math::Vec3<NumType>,
    //pub rot_mat: crate::math::Mat33<NumType>,
    pub pos: nalgebra::Vector3<NumType>,
    pub lin_vel: nalgebra::Vector3<NumType>,
    pub rot_mat: nalgebra::Matrix3<NumType>,
    pub ang_mom: nalgebra::Vector3<NumType>,
    pub inv_ine: nalgebra::Matrix3<NumType>,
}

impl<NumType> RigidBody<NumType>
where
    NumType: nalgebra::Scalar
        + ops::Mul
        + ops::MulAssign
        + ops::AddAssign
        + ops::Sub<Output = NumType>
        + ops::SubAssign
        + num_traits::identities::Zero
        + num_traits::One,
    nalgebra::Vector3<NumType>: ops::Mul<
            NumType,
            Output = nalgebra::Matrix<
                NumType,
                nalgebra::Const<3>,
                nalgebra::Const<1>,
                <nalgebra::DefaultAllocator as nalgebra::allocator::Allocator<
                    nalgebra::Const<3>,
                    nalgebra::Const<1>,
                >>::Buffer<NumType>,
            >,
        > + ops::AddAssign,
{
    pub fn step_sim(&mut self, delta_time: NumType) {
        let omega: nalgebra::Vector3<NumType> =
            &self.rot_mat * &self.inv_ine * &self.rot_mat.transpose() * &self.ang_mom;
        self.rot_mat += &nalgebra::Matrix3::from_columns(&[
            omega.cross(&self.rot_mat.column(0)),
            omega.cross(&self.rot_mat.column(1)),
            omega.cross(&self.rot_mat.column(0)),
        ]) * delta_time.clone();

        self.pos += self.lin_vel.clone() * delta_time;
    }
}
