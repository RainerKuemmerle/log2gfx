extern crate nalgebra as na;

#[derive(Debug, Clone, Copy)]
pub struct MapCreatorParameter {
    ///< the max range of the laser scanner data
    pub max_range: f64,
    ///< max usable range of the laser data
    pub max_usable_range: f64,
    ///< resolution of the map
    pub resolution: f64,
    ///< offset of the map, can be automatically set to zero
    pub offset: na::Isometry2<f64>,
    ///< border around the map which is set to unknown
    pub border: f64,
    ///< set the first pose of the map automatically to zero
    pub zero_first_pose: bool,
    ///< print some verbose information while creating the map
    pub verbose: bool,
}

impl Default for MapCreatorParameter {
    fn default() -> Self {
        Self {
            max_range: 20.,
            max_usable_range: 20.,
            resolution: 0.1,
            offset: na::Isometry2::identity(),
            border: 2.0,
            zero_first_pose: false,
            verbose: false,
        }
    }
}
