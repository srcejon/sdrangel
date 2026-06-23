///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#include "cameraplatesolverinternal.h"

// Levenberg-Marquardt pose refinement + rotation-vector orientation helpers (WS5).

auto CameraPlateSolver::SolverContext::normalizeSignedDegrees(double value) -> double
{
    value = normalizeDegrees(value);
    if (value > 180.0) {
        value -= 360.0;
    }
    return value;
}

auto CameraPlateSolver::SolverContext::poseFromEvaluation(const Evaluation& evaluation) -> PlateSolveLmPose
{
    PlateSolveLmPose pose;
    pose.azimuthDegrees = evaluation.azimuthDegrees;
    pose.elevationDegrees = evaluation.elevationDegrees;
    pose.rollDegrees = evaluation.rollDegrees;
    pose.fovDegrees = evaluation.fovDegrees;
    pose.centerOffsetXPixels = evaluation.centerOffsetXPixels;
    pose.centerOffsetYPixels = evaluation.centerOffsetYPixels;
    pose.distortionK1 = evaluation.distortionK1;
    return pose;
}

auto CameraPlateSolver::SolverContext::clampPlateSolveLmPose(const QSize& imageSize, PlateSolveLmPose& pose) -> void
{
    pose.azimuthDegrees = normalizeDegrees(pose.azimuthDegrees);
    pose.elevationDegrees = std::clamp(pose.elevationDegrees, kVisibleAltitudeFloor, 90.0);
    pose.rollDegrees = normalizeSignedDegrees(pose.rollDegrees);
    pose.fovDegrees = std::clamp(
        pose.fovDegrees,
        static_cast<double>(CameraSettings::m_minFov),
        static_cast<double>(CameraSettings::m_maxFov));
    const double maxCenterOffset = std::max(1.0, static_cast<double>(std::max(imageSize.width(), imageSize.height())));
    pose.centerOffsetXPixels = std::clamp(pose.centerOffsetXPixels, -maxCenterOffset, maxCenterOffset);
    pose.centerOffsetYPixels = std::clamp(pose.centerOffsetYPixels, -maxCenterOffset, maxCenterOffset);
    pose.distortionK1 = std::clamp(pose.distortionK1, -0.75, 0.75);
}

auto CameraPlateSolver::SolverContext::rotVecLmEnabled() -> bool
{
    // WS2: rotation-vector LM orientation is the DEFAULT. Kill-switch
    // SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_ROTVEC_LM reverts to the legacy az/el/roll path for A/B.
    static const bool disabled = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_ROTVEC_LM");
    return !disabled;
}

auto CameraPlateSolver::SolverContext::rotVecLmActive(const CameraSettings& settings) -> bool
{
    return rotVecLmEnabled() && plateSolveStartUsesDirection(settings);
}

auto CameraPlateSolver::SolverContext::lmBasisFromAzElRoll(double azimuthDegrees, double elevationDegrees, double rollDegrees, SkyVector& center, SkyVector& right, SkyVector& up) -> void
{
    const double azr = degToRad(azimuthDegrees);
    center = normalize(vectorFromAltAz(azimuthDegrees, elevationDegrees));
    right = normalize({std::cos(azr), -std::sin(azr), 0.0});
    up = normalize(cross(right, center));
    const double rr = degToRad(rollDegrees);
    if (std::fabs(rr) > 1e-9) {
        right = normalize(rotateAroundAxis(right, center, rr));
        up = normalize(rotateAroundAxis(up, center, rr));
    }
}

auto CameraPlateSolver::SolverContext::lmAzElRollFromBasis(const SkyVector& center, const SkyVector& right, double& azimuthDegrees, double& elevationDegrees, double& rollDegrees) -> void
{
    const SkyVector c = normalize(center);
    elevationDegrees = std::asin(std::clamp(c.z, -1.0, 1.0)) * (180.0 / kPi);
    azimuthDegrees = normalizeDegrees(std::atan2(c.x, c.y) * (180.0 / kPi));
    const double azr = degToRad(azimuthDegrees);
    const SkyVector right0 = normalize({std::cos(azr), -std::sin(azr), 0.0});
    const SkyVector r = normalize(right);
    // Signed roll: angle from the unrolled right vector to r, measured about the boresight (center).
    const double cosA = std::clamp(dot(right0, r), -1.0, 1.0);
    const double sinA = dot(cross(right0, r), c);
    rollDegrees = normalizeSignedDegrees(std::atan2(sinA, cosA) * (180.0 / kPi));
}

auto CameraPlateSolver::SolverContext::lmRotateOrientationCameraFrame(double& azimuthDegrees, double& elevationDegrees, double& rollDegrees, int cameraAxis, double deltaDegrees) -> void
{
    SkyVector center, right, up;
    lmBasisFromAzElRoll(azimuthDegrees, elevationDegrees, rollDegrees, center, right, up);
    const SkyVector axis = (cameraAxis == 0) ? right : (cameraAxis == 1) ? up : center;
    const double a = degToRad(deltaDegrees);
    center = normalize(rotateAroundAxis(center, axis, a));
    right = normalize(rotateAroundAxis(right, axis, a));
    lmAzElRollFromBasis(center, right, azimuthDegrees, elevationDegrees, rollDegrees);
}

auto CameraPlateSolver::SolverContext::addPlateSolveLmParameterDelta(const QSize& imageSize, PlateSolveLmPose& pose, PlateSolveLmParameter parameter, double delta, bool useRotVec) -> void
{
    const bool rotVec = useRotVec;
    switch (parameter)
    {
    case PlateSolveLmAzimuth:
        if (rotVec) {
            lmRotateOrientationCameraFrame(pose.azimuthDegrees, pose.elevationDegrees, pose.rollDegrees, 1, delta);
        } else {
            pose.azimuthDegrees += delta;
        }
        break;
    case PlateSolveLmElevation:
        if (rotVec) {
            lmRotateOrientationCameraFrame(pose.azimuthDegrees, pose.elevationDegrees, pose.rollDegrees, 0, delta);
        } else {
            pose.elevationDegrees += delta;
        }
        break;
    case PlateSolveLmRoll:
        if (rotVec) {
            lmRotateOrientationCameraFrame(pose.azimuthDegrees, pose.elevationDegrees, pose.rollDegrees, 2, delta);
        } else {
            pose.rollDegrees += delta;
        }
        break;
    case PlateSolveLmFov:
        pose.fovDegrees += delta;
        break;
    case PlateSolveLmCenterX:
        pose.centerOffsetXPixels += delta;
        break;
    case PlateSolveLmCenterY:
        pose.centerOffsetYPixels += delta;
        break;
    case PlateSolveLmDistortionK1:
        pose.distortionK1 += delta;
        break;
    default:
        break;
    }
    clampPlateSolveLmPose(imageSize, pose);
}

auto CameraPlateSolver::SolverContext::robustPlateSolveLmWeight(double residualNormPixels, double thresholdPixels) -> double
{
    if ((residualNormPixels <= thresholdPixels) || (residualNormPixels <= 1e-9)) {
        return 1.0;
    }
    return thresholdPixels / residualNormPixels;
}

auto CameraPlateSolver::SolverContext::solveSmallLinearSystem(double matrix[PlateSolveLmParameterCount][PlateSolveLmParameterCount], double rhs[PlateSolveLmParameterCount], double solution[PlateSolveLmParameterCount], int size) -> bool
{
    for (int i = 0; i < size; ++i) {
        solution[i] = 0.0;
    }
    for (int column = 0; column < size; ++column)
    {
        int pivotRow = column;
        double pivotAbs = std::fabs(matrix[column][column]);
        for (int row = column + 1; row < size; ++row)
        {
            const double candidateAbs = std::fabs(matrix[row][column]);
            if (candidateAbs > pivotAbs)
            {
                pivotAbs = candidateAbs;
                pivotRow = row;
            }
        }
        if (pivotAbs < 1e-12) {
            return false;
        }
        if (pivotRow != column)
        {
            for (int col = column; col < size; ++col) {
                std::swap(matrix[column][col], matrix[pivotRow][col]);
            }
            std::swap(rhs[column], rhs[pivotRow]);
        }

        const double pivot = matrix[column][column];
        for (int col = column; col < size; ++col) {
            matrix[column][col] /= pivot;
        }
        rhs[column] /= pivot;

        for (int row = 0; row < size; ++row)
        {
            if (row == column) {
                continue;
            }
            const double factor = matrix[row][column];
            if (std::fabs(factor) <= 0.0) {
                continue;
            }
            for (int col = column; col < size; ++col) {
                matrix[row][col] -= factor * matrix[column][col];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    for (int i = 0; i < size; ++i) {
        solution[i] = rhs[i];
    }
    return true;
}

