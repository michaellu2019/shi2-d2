// -*- coding: utf-8 -*-
// Copyright (C) 2012 Rosen Diankov <rosen.diankov@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/** \brief  Header file for all ikfast c++ files/shared objects.

    The ikfast inverse kinematics compiler is part of OpenRAVE.

    The file is divided into two sections:
    - <b>Common</b> - the abstract classes section that all ikfast share regardless of their settings
    - <b>Library Specific</b> - the library-specific definitions, which depends on the precision/settings that the
   library was compiled with

    The defines are as follows, they are also used for the ikfast C++ class:

   - IKFAST_HEADER_COMMON - common classes
   - IKFAST_HAS_LIBRARY - if defined, will include library-specific functions. by default this is off
   - IKFAST_CLIBRARY - Define this linking statically or dynamically to get correct visibility.
   - IKFAST_NO_MAIN - Remove the ``main`` function, usually used with IKFAST_CLIBRARY
   - IKFAST_ASSERT - Define in order to get a custom assert called when NaNs, divides by zero, and other invalid
   conditions are detected.
   - IKFAST_REAL - Use to force a custom real number type for IkReal.
   - IKFAST_NAMESPACE - Enclose all functions and classes in this namespace, the ``main`` function is excluded.

 */

#pragma once

#include <vector>
#include <list>
#include <stdexcept>

#pragma once

/// should be the same as ikfast.__version__
#define IKFAST_VERSION 61

namespace ikfast
{
/// \brief holds the solution for a single dof
template <typename T>
class IkSingleDOFSolutionBase
{
public:
  IkSingleDOFSolutionBase() : fmul(0), foffset(0), freeind(-1), maxsolutions(1)
  {
    indices[0] = indices[1] = indices[2] = indices[3] = indices[4] = -1;
  }
  T fmul, foffset;             ///< joint value is fmul*sol[freeind]+foffset
  signed char freeind;         ///< if >= 0, mimics another joint
  unsigned char jointtype;     ///< joint type, 0x01 is revolute, 0x11 is slider
  unsigned char maxsolutions;  ///< max possible indices, 0 if controlled by free index or a free joint itself
  unsigned char indices[5];  ///< unique index of the solution used to keep track on what part it came from. sometimes a
  /// solution can be repeated for different indices. store at least another repeated root
};

/// \brief The discrete solutions are returned in this structure.
///
/// Sometimes the joint axes of the robot can align allowing an infinite number of solutions.
/// Stores all these solutions in the form of free variables that the user has to set when querying the solution. Its
/// prototype is:
template <typename T>
class IkSolutionBase
{
public:
  virtual ~IkSolutionBase()
  {
  }
  /// \brief gets a concrete solution
  ///
  /// \param[out] solution the result
  /// \param[in] freevalues values for the free parameters \se GetFree
  virtual void GetSolution(T* solution, const T* freevalues) const = 0;

  /// \brief std::vector version of \ref GetSolution
  virtual void GetSolution(std::vector<T>& solution, const std::vector<T>& freevalues) const
  {
    solution.resize(GetDOF());
    GetSolution(&solution.at(0), freevalues.size() > 0 ? &freevalues.at(0) : nullptr);
  }

  /// \brief Gets the indices of the configuration space that have to be preset before a full solution can be returned
  ///
  /// \return vector of indices indicating the free parameters
  virtual const std::vector<int>& GetFree() const = 0;

  /// \brief the dof of the solution
  virtual int GetDOF() const = 0;
};

/// \brief manages all the solutions
template <typename T>
class IkSolutionListBase
{
public:
  virtual ~IkSolutionListBase()
  {
  }

  /// \brief add one solution and return its index for later retrieval
  ///
  /// \param vinfos Solution data for each degree of freedom of the manipulator
  /// \param vfree If the solution represents an infinite space, holds free parameters of the solution that users can
  /// freely set.
  virtual size_t AddSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree) = 0;

  /// \brief returns the solution pointer
  virtual const IkSolutionBase<T>& GetSolution(size_t index) const = 0;

  /// \brief returns the number of solutions stored
  virtual size_t GetNumSolutions() const = 0;

  /// \brief clears all current solutions, note that any memory addresses returned from \ref GetSolution will be
  /// invalidated.
  virtual void Clear() = 0;
};

/// \brief holds function pointers for all the exported functions of ikfast
template <typename T>
class IkFastFunctions
{
public:
  IkFastFunctions()
    : _ComputeIk(nullptr)
    , _ComputeFk(nullptr)
    , _GetNumFreeParameters(nullptr)
    , _GetFreeParameters(nullptr)
    , _GetNumJoints(nullptr)
    , _GetIkRealSize(nullptr)
    , _GetIkFastVersion(nullptr)
    , _GetIkType(nullptr)
    , _GetKinematicsHash(nullptr)
  {
  }
  virtual ~IkFastFunctions()
  {
  }
  typedef bool (*ComputeIkFn)(const T*, const T*, const T*, IkSolutionListBase<T>&);
  ComputeIkFn _ComputeIk;
  typedef void (*ComputeFkFn)(const T*, T*, T*);
  ComputeFkFn _ComputeFk;
  typedef int (*GetNumFreeParametersFn)();
  GetNumFreeParametersFn _GetNumFreeParameters;
  typedef int* (*GetFreeParametersFn)();
  GetFreeParametersFn _GetFreeParameters;
  typedef int (*GetNumJointsFn)();
  GetNumJointsFn _GetNumJoints;
  typedef int (*GetIkRealSizeFn)();
  GetIkRealSizeFn _GetIkRealSize;
  typedef const char* (*GetIkFastVersionFn)();
  GetIkFastVersionFn _GetIkFastVersion;
  typedef int (*GetIkTypeFn)();
  GetIkTypeFn _GetIkType;
  typedef const char* (*GetKinematicsHashFn)();
  GetKinematicsHashFn _GetKinematicsHash;
};

// Implementations of the abstract classes, user doesn't need to use them

/// \brief Default implementation of \ref IkSolutionBase
template <typename T>
class IkSolution : public IkSolutionBase<T>
{
public:
  IkSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree)
  {
    _vbasesol = vinfos;
    _vfree = vfree;
  }

  virtual void GetSolution(T* solution, const T* freevalues) const
  {
    for (std::size_t i = 0; i < _vbasesol.size(); ++i)
    {
      if (_vbasesol[i].freeind < 0)
        solution[i] = _vbasesol[i].foffset;
      else
      {
        solution[i] = freevalues[_vbasesol[i].freeind] * _vbasesol[i].fmul + _vbasesol[i].foffset;
        if (solution[i] > T(3.14159265358979))
        {
          solution[i] -= T(6.28318530717959);
        }
        else if (solution[i] < T(-3.14159265358979))
        {
          solution[i] += T(6.28318530717959);
        }
      }
    }
  }

  virtual void GetSolution(std::vector<T>& solution, const std::vector<T>& freevalues) const
  {
    solution.resize(GetDOF());
    GetSolution(&solution.at(0), freevalues.size() > 0 ? &freevalues.at(0) : nullptr);
  }

  virtual const std::vector<int>& GetFree() const
  {
    return _vfree;
  }
  virtual int GetDOF() const
  {
    return static_cast<int>(_vbasesol.size());
  }

  virtual void Validate() const
  {
    for (size_t i = 0; i < _vbasesol.size(); ++i)
    {
      if (_vbasesol[i].maxsolutions == (unsigned char)-1)
      {
        throw std::runtime_error("max solutions for joint not initialized");
      }
      if (_vbasesol[i].maxsolutions > 0)
      {
        if (_vbasesol[i].indices[0] >= _vbasesol[i].maxsolutions)
        {
          throw std::runtime_error("index >= max solutions for joint");
        }
        if (_vbasesol[i].indices[1] != (unsigned char)-1 && _vbasesol[i].indices[1] >= _vbasesol[i].maxsolutions)
        {
          throw std::runtime_error("2nd index >= max solutions for joint");
        }
      }
    }
  }

  virtual void GetSolutionIndices(std::vector<unsigned int>& v) const
  {
    v.resize(0);
    v.push_back(0);
    for (int i = static_cast<int>(_vbasesol.size()) - 1; i >= 0; --i)
    {
      if (_vbasesol[i].maxsolutions != (unsigned char)-1 && _vbasesol[i].maxsolutions > 1)
      {
        for (size_t j = 0; j < v.size(); ++j)
        {
          v[j] *= _vbasesol[i].maxsolutions;
        }
        size_t orgsize = v.size();
        if (_vbasesol[i].indices[1] != (unsigned char)-1)
        {
          for (size_t j = 0; j < orgsize; ++j)
          {
            v.push_back(v[j] + _vbasesol[i].indices[1]);
          }
        }
        if (_vbasesol[i].indices[0] != (unsigned char)-1)
        {
          for (size_t j = 0; j < orgsize; ++j)
          {
            v[j] += _vbasesol[i].indices[0];
          }
        }
      }
    }
  }

  std::vector<IkSingleDOFSolutionBase<T> > _vbasesol;  ///< solution and their offsets if joints are mimiced
  std::vector<int> _vfree;
};

/// \brief Default implementation of \ref IkSolutionListBase
template <typename T>
class IkSolutionList : public IkSolutionListBase<T>
{
public:
  virtual size_t AddSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree)
  {
    size_t index = _listsolutions.size();
    _listsolutions.push_back(IkSolution<T>(vinfos, vfree));
    return index;
  }

  virtual const IkSolutionBase<T>& GetSolution(size_t index) const
  {
    if (index >= _listsolutions.size())
    {
      throw std::runtime_error("GetSolution index is invalid");
    }
    typename std::list<IkSolution<T> >::const_iterator it = _listsolutions.begin();
    std::advance(it, index);
    return *it;
  }

  virtual size_t GetNumSolutions() const
  {
    return _listsolutions.size();
  }

  virtual void Clear()
  {
    _listsolutions.clear();
  }

protected:
  std::list<IkSolution<T> > _listsolutions;
};
}  // namespace ikfast

// The following code is dependent on the C++ library linking with.
#define IKFAST_HAS_LIBRARY 1
#ifdef IKFAST_HAS_LIBRARY

// defined when creating a shared object/dll
#ifdef IKFAST_CLIBRARY
#ifdef _MSC_VER
#define IKFAST_API extern "C" __declspec(dllexport)
#else
#define IKFAST_API extern "C"
#endif
#else
#define IKFAST_API
#endif

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE
{
#endif

#ifdef IKFAST_REAL
typedef IKFAST_REAL IkReal;
#else
typedef double IkReal;
#endif

/** \brief Computes all IK solutions given a end effector coordinates and the free joints.

   - ``eetrans`` - 3 translation values. For iktype **TranslationXYOrientation3D**, the z-axis is the orientation.
   - ``eerot``
   - For **Transform6D** it is 9 values for the 3x3 rotation matrix.
   - For **Direction3D**, **Ray4D**, and **TranslationDirection5D**, the first 3 values represent the target direction.
   - For **TranslationXAxisAngle4D**, **TranslationYAxisAngle4D**, and **TranslationZAxisAngle4D** the first value
   represents the angle.
   - For **TranslationLocalGlobal6D**, the diagonal elements ([0],[4],[8]) are the local translation inside the end
   effector coordinate system.
 */
IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree,
                          ikfast::IkSolutionListBase<IkReal>& solutions);

/// \brief Computes the end effector coordinates given the joint values. This function is used to double check ik.
IKFAST_API void ComputeFk(const IkReal* joints, IkReal* eetrans, IkReal* eerot);

/// \brief returns the number of free parameters users has to set apriori
IKFAST_API int GetNumFreeParameters();

/// \brief the indices of the free parameters indexed by the chain joints
IKFAST_API int* GetFreeParameters();

/// \brief the total number of indices of the chain
IKFAST_API int GetNumJoints();

/// \brief the size in bytes of the configured number type
IKFAST_API int GetIkRealSize();

/// \brief the ikfast version used to generate this file
IKFAST_API const char* GetIkFastVersion();

/// \brief the ik type ID
IKFAST_API int GetIkType();

/// \brief a hash of all the chain values used for double checking that the correct IK is used.
IKFAST_API const char* GetKinematicsHash();

#ifdef IKFAST_NAMESPACE
}
#endif

#endif  // IKFAST_HAS_LIBRARY

#include <rclcpp/rclcpp.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace moveit::core;

// Need a floating point tolerance when checking joint limits, in case the joint starts at limit
const double LIMIT_TOLERANCE = .0000001;
/// \brief Search modes for searchPositionIK(), see there
enum SEARCH_MODE
{
  OPTIMIZE_FREE_JOINT = 1,
  OPTIMIZE_MAX_JOINT = 2
};

// namespace shi2d2_left_leg_group
// {
#define IKFAST_NO_MAIN  // Don't include main() from IKFast

/// \brief The types of inverse kinematics parameterizations supported.
///
/// The minimum degree of freedoms required is set in the upper 4 bits of each type.
/// The number of values used to represent the parameterization ( >= dof ) is the next 4 bits.
/// The lower bits contain a unique id of the type.
enum IkParameterizationType
{
  IKP_None = 0,
  IKP_Transform6D = 0x67000001,    ///< end effector reaches desired 6D transformation
  IKP_Rotation3D = 0x34000002,     ///< end effector reaches desired 3D rotation
  IKP_Translation3D = 0x33000003,  ///< end effector origin reaches desired 3D translation
  IKP_Direction3D = 0x23000004,    ///< direction on end effector coordinate system reaches desired direction
  IKP_Ray4D = 0x46000005,          ///< ray on end effector coordinate system reaches desired global ray
  IKP_Lookat3D = 0x23000006,       ///< direction on end effector coordinate system points to desired 3D position
  IKP_TranslationDirection5D = 0x56000007,  ///< end effector origin and direction reaches desired 3D translation and
  /// direction. Can be thought of as Ray IK where the origin of the ray must
  /// coincide.
  IKP_TranslationXY2D = 0x22000008,             ///< 2D translation along XY plane
  IKP_TranslationXYOrientation3D = 0x33000009,  ///< 2D translation along XY plane and 1D rotation around Z axis. The
  /// offset of the rotation is measured starting at +X, so at +X is it 0,
  /// at +Y it is pi/2.
  IKP_TranslationLocalGlobal6D = 0x3600000a,  ///< local point on end effector origin reaches desired 3D global point

  IKP_TranslationXAxisAngle4D = 0x4400000b,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with x-axis  like a cone, angle is from
  /// 0-pi. Axes defined in the manipulator base link's coordinate system)
  IKP_TranslationYAxisAngle4D = 0x4400000c,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with y-axis  like a cone, angle is from
  /// 0-pi. Axes defined in the manipulator base link's coordinate system)
  IKP_TranslationZAxisAngle4D = 0x4400000d,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with z-axis like a cone, angle is from
  /// 0-pi. Axes are defined in the manipulator base link's coordinate system.

  IKP_TranslationXAxisAngleZNorm4D = 0x4400000e,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to z-axis and be rotated at a
  /// certain angle starting from the x-axis (defined in the manipulator
  /// base link's coordinate system)
  IKP_TranslationYAxisAngleXNorm4D = 0x4400000f,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to x-axis and be rotated at a
  /// certain angle starting from the y-axis (defined in the manipulator
  /// base link's coordinate system)
  IKP_TranslationZAxisAngleYNorm4D = 0x44000010,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to y-axis and be rotated at a
  /// certain angle starting from the z-axis (defined in the manipulator
  /// base link's coordinate system)

  IKP_NumberOfParameterizations = 16,  ///< number of parameterizations (does not count IKP_None)

  IKP_VelocityDataBit =
      0x00008000,  ///< bit is set if the data represents the time-derivate velocity of an IkParameterization
  IKP_Transform6DVelocity = IKP_Transform6D | IKP_VelocityDataBit,
  IKP_Rotation3DVelocity = IKP_Rotation3D | IKP_VelocityDataBit,
  IKP_Translation3DVelocity = IKP_Translation3D | IKP_VelocityDataBit,
  IKP_Direction3DVelocity = IKP_Direction3D | IKP_VelocityDataBit,
  IKP_Ray4DVelocity = IKP_Ray4D | IKP_VelocityDataBit,
  IKP_Lookat3DVelocity = IKP_Lookat3D | IKP_VelocityDataBit,
  IKP_TranslationDirection5DVelocity = IKP_TranslationDirection5D | IKP_VelocityDataBit,
  IKP_TranslationXY2DVelocity = IKP_TranslationXY2D | IKP_VelocityDataBit,
  IKP_TranslationXYOrientation3DVelocity = IKP_TranslationXYOrientation3D | IKP_VelocityDataBit,
  IKP_TranslationLocalGlobal6DVelocity = IKP_TranslationLocalGlobal6D | IKP_VelocityDataBit,
  IKP_TranslationXAxisAngle4DVelocity = IKP_TranslationXAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationYAxisAngle4DVelocity = IKP_TranslationYAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationZAxisAngle4DVelocity = IKP_TranslationZAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationXAxisAngleZNorm4DVelocity = IKP_TranslationXAxisAngleZNorm4D | IKP_VelocityDataBit,
  IKP_TranslationYAxisAngleXNorm4DVelocity = IKP_TranslationYAxisAngleXNorm4D | IKP_VelocityDataBit,
  IKP_TranslationZAxisAngleYNorm4DVelocity = IKP_TranslationZAxisAngleYNorm4D | IKP_VelocityDataBit,

  IKP_UniqueIdMask = 0x0000ffff,   ///< the mask for the unique ids
  IKP_CustomDataBit = 0x00010000,  ///< bit is set if the ikparameterization contains custom data, this is only used
  /// when serializing the ik parameterizations
};

// struct for storing and sorting solutions
struct LimitObeyingSol
{
  std::vector<double> value;
  double dist_from_seed;

  bool operator<(const LimitObeyingSol& a) const
  {
    return dist_from_seed < a.dist_from_seed;
  }
};

class IKFastKinematicsPlugin : public kinematics::KinematicsBase
{
  std::vector<std::string> joint_names_;
  std::vector<double> joint_min_vector_;
  std::vector<double> joint_max_vector_;
  std::vector<bool> joint_has_limits_vector_;
  std::vector<std::string> link_names_;
  const size_t num_joints_;
  std::vector<int> free_params_;

  // The ikfast and base frame are the start and end of the kinematic chain for which the
  // IKFast analytic solution was generated.
  const std::string IKFAST_TIP_FRAME_ = "left_foot_link";
  const std::string IKFAST_BASE_FRAME_ = "left_upper_hip_link";

  // prefix added to tip- and baseframe to allow different namespaces or multi-robot setups
  std::string link_prefix_;

  // The transform tip and base bool are set to true if this solver is used with a kinematic
  // chain that extends beyond the ikfast tip and base frame. The solution will be valid so
  // long as there are no active, passive, or mimic joints between either the ikfast_tip_frame
  // and the tip_frame of the group or the ikfast_base_frame and the base_frame for the group.
  bool tip_transform_required_;
  bool base_transform_required_;

  // We store the transform from the ikfast_base_frame to the group base_frame as well as the
  // ikfast_tip_frame to the group tip_frame to transform input poses into the solver frame.
  Eigen::Isometry3d chain_base_to_group_base_;
  Eigen::Isometry3d group_tip_to_chain_tip_;

  bool initialized_;  // Internal variable that indicates whether solvers are configured and ready
  const rclcpp::Logger LOGGER = rclcpp::get_logger("shi2d2_left_leg_group_ikfast_solver");

  const std::vector<std::string>& getJointNames() const override
  {
    return joint_names_;
  }
  const std::vector<std::string>& getLinkNames() const override
  {
    return link_names_;
  }

public:
  /** @class
   *  @brief Interface for an IKFast kinematics plugin
   */
  IKFastKinematicsPlugin() : num_joints_(GetNumJoints()), initialized_(false)
  {
    srand(time(nullptr));
    supported_methods_.push_back(kinematics::DiscretizationMethods::NO_DISCRETIZATION);
    supported_methods_.push_back(kinematics::DiscretizationMethods::ALL_DISCRETIZED);
    supported_methods_.push_back(kinematics::DiscretizationMethods::ALL_RANDOM_SAMPLED);
  }

  /**
   * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @return True if a valid solution was found, false otherwise
   */

  // Returns the IK solution that is within joint limits closest to ik_seed_state
  bool
  getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, compute the set joint angles solutions that are able to reach it.
   *
   * This is a default implementation that returns only one solution and so its result is equivalent to calling
   * 'getPositionIK(...)' with a zero initialized seed.
   *
   * @param ik_poses  The desired pose of each tip link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solutions A vector of vectors where each entry is a valid joint solution
   * @param result A struct that reports the results of the query
   * @param options An option struct which contains the type of redundancy discretization used. This default
   *                implementation only supports the KinmaticSearches::NO_DISCRETIZATION method; requesting any
   *                other will result in failure.
   * @return True if a valid set of solutions was found, false otherwise.
   */
  bool getPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                     std::vector<std::vector<double>>& solutions, kinematics::KinematicsResult& result,
                     const kinematics::KinematicsQueryOptions& options) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
   * around those specified in the seed state are admissible and need to be searched.
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param consistency_limit the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a set of joint angles and a set of links, compute their pose
   *
   * @param link_names A set of links for which FK needs to be computed
   * @param joint_angles The state for which FK is being computed
   * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
   * @return True if a valid solution was found, false otherwise
   */
  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override;

  /**
   * @brief Sets the discretization value for the redundant joint.
   *
   * Since this ikfast implementation allows for one redundant joint then only the first entry will be in the
   *discretization map will be used.
   * Calling this method replaces previous discretization settings.
   *
   * @param discretization a map of joint indices and discretization value pairs.
   */
  void setSearchDiscretization(const std::map<unsigned int, double>& discretization);

  /**
   * @brief Overrides the default method to prevent changing the redundant joints
   */
  bool setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices) override;

private:
  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, const std::string& base_frame,
                  const std::vector<std::string>& tip_frames, double search_discretization) override;

  /**
   * @brief Calls the IK solver from IKFast
   * @return The number of solutions found
   */
  size_t solve(KDL::Frame& pose_frame, const std::vector<double>& vfree, ikfast::IkSolutionList<IkReal>& solutions) const;

  /**
   * @brief Gets a specific solution from the set
   */
  void getSolution(const ikfast::IkSolutionList<IkReal>& solutions, int i, std::vector<double>& solution) const;

  /**
   * @brief Gets a specific solution from the set with joints rotated 360° to be near seed state where possible
   */
  void getSolution(const ikfast::IkSolutionList<IkReal>& solutions, const std::vector<double>& ik_seed_state, int i,
                   std::vector<double>& solution) const;

  /**
   * @brief If the value is outside of min/max then it tries to +/- 2 * pi to put the value into the range
   */
  double enforceLimits(double val, double min, double max) const;

  void fillFreeParams(int count, int* array);
  bool getCount(int& count, const int& max_count, const int& min_count) const;

  /**
   * @brief samples the designated redundant joint using the chosen discretization method
   * @param  method              An enumeration flag indicating the discretization method to be used
   * @param  sampled_joint_vals  Sampled joint values for the redundant joint
   * @return True if sampling succeeded.
   */
  bool sampleRedundantJoint(kinematics::DiscretizationMethod method, std::vector<double>& sampled_joint_vals) const;

  /// Validate that we can compute a fixed transform between from and to links.
  bool computeRelativeTransform(const std::string& from, const std::string& to, Eigen::Isometry3d& transform,
                                bool& differs_from_identity);
  /**
   * @brief  Transforms the input pose to the correct frame for the solver. This assumes that the group includes the
   * entire solver chain and that any joints outside of the solver chain within the group are are fixed.
   * @param  ik_pose             The pose to be transformed which should be in the correct frame for the group.
   * @param  ik_pose_chain       The ik_pose to be populated with the appropriate pose for the solver
   */
  void transformToChainFrame(const geometry_msgs::msg::Pose& ik_pose, KDL::Frame& ik_pose_chain) const;
};  // end class

// namespace shi2d2_left_leg_group
// {
class A
{
  public:
    A();
    int get_id();
    void double_id();

  private:
    int id_;
};
// }  // namespace shi2d2_left_leg_group