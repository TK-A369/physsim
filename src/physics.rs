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
        + num_traits::One
        + num_traits::real::Real,
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
        //Reorthonormalize
        let rot_mat_x = self.rot_mat.column(0);
        let rot_mat_x = &rot_mat_x
            * (NumType::one()
                / (rot_mat_x.x * rot_mat_x.x
                    + rot_mat_x.y * rot_mat_x.y
                    + rot_mat_x.z * rot_mat_x.z)
                    .sqrt());
        let rot_mat_y = self.rot_mat.column(2).cross(&self.rot_mat.column(0));
        let rot_mat_y = &rot_mat_y
            * (NumType::one()
                / (rot_mat_y.x * rot_mat_y.x
                    + rot_mat_y.y * rot_mat_y.y
                    + rot_mat_y.z * rot_mat_y.z)
                    .sqrt());
        let rot_mat_z = self.rot_mat.column(0).cross(&self.rot_mat.column(1));
        let rot_mat_z = &rot_mat_z
            * (NumType::one()
                / (rot_mat_z.x * rot_mat_z.x
                    + rot_mat_z.y * rot_mat_z.y
                    + rot_mat_z.z * rot_mat_z.z)
                    .sqrt());
        self.rot_mat = nalgebra::Matrix3::from_columns(&[rot_mat_x, rot_mat_y, rot_mat_z]);

        self.pos += self.lin_vel.clone() * delta_time;
    }
}
