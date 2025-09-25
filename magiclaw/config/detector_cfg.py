#!/usr/bin/env python

"""
Detector configuration
===

This module contains the configuration for the detector.
"""

import os
import yaml


class DetectorConfig:
    """
    Detector configuration class.
    
    This class is used to configure the detector parameters such as adaptive thresholding,
    marker perimeter rates, polygonal approximation accuracy, corner distance rates, and more.
    
    Attributes:
        adaptiveThreshConstant (int): The constant used in adaptive thresholding.
        adaptiveThreshWinSizeMin (int): The minimum window size for adaptive thresholding.
        adaptiveThreshWinSizeMax (int): The maximum window size for adaptive thresholding.
        adaptiveThreshWinSizeStep (int): The step size for the window size in adaptive thresholding.
        minMarkerPerimeterRate (float): The minimum marker perimeter rate.
        maxMarkerPerimeterRate (float): The maximum marker perimeter rate.
        polygonalApproxAccuracyRate (float): The accuracy rate for polygonal approximation.
        minCornerDistanceRate (float): The minimum corner distance rate.
        minDistanceToBorder (int): The minimum distance to the border.
        minMarkerDistanceRate (float): The minimum marker distance rate.
        markerBorderBits (int): The number of bits in the marker border.
        perspectiveRemovePixelPerCell (int): The number of pixels per cell for perspective removal.
        perspectiveRemoveIgnoredMarginPerCell (float): The ignored margin per cell for perspective removal.
        maxErroneousBitsInBorderRate (float): The maximum erroneous bits in the border rate.
        minOtsuStdDev (float): The minimum Otsu standard deviation.
        cornerRefinementMethod (int): The method used for corner refinement.
        cornerRefinementWinSize (int): The window size for corner refinement.
        cornerRefinementMaxIterations (float): The maximum iterations for corner refinement.
        cornerRefinementMinAccuracy (float): The minimum accuracy for corner refinement.
        errorCorrectionRate (float): The error correction rate.
        useAruco3Detection (int): Whether to use ArUco3 detection or not.
        minSideLengthCanonicalImg (int): The minimum side length of the canonical image.
        minMarkerLengthRatioOriginalImg (float): The minimum marker length ratio in the original image
    """

    def __init__(
        self,
        adaptiveThreshConstant: int = 7,
        adaptiveThreshWinSizeMin: int = 3,
        adaptiveThreshWinSizeMax: int = 23,
        adaptiveThreshWinSizeStep: int = 5,
        minMarkerPerimeterRate: float = 0.05,
        maxMarkerPerimeterRate: float = 3.0,
        polygonalApproxAccuracyRate: float = 0.01,
        minCornerDistanceRate: float = 0.05,
        minDistanceToBorder: int = 1,
        minMarkerDistanceRate: float = 0.05,
        markerBorderBits: int = 1,
        perspectiveRemovePixelPerCell: int = 8,
        perspectiveRemoveIgnoredMarginPerCell: float = 0.13,
        maxErroneousBitsInBorderRate: float = 0.04,
        minOtsuStdDev: float = 5.0,
        cornerRefinementMethod: int = 2,
        cornerRefinementWinSize: int = 5,
        cornerRefinementMaxIterations: float = 50,
        cornerRefinementMinAccuracy: float = 0.001,
        errorCorrectionRate: float = 0.1,
        useAruco3Detection: int = 1,
        minSideLengthCanonicalImg: int = 16,
        minMarkerLengthRatioOriginalImg: float = 0.05,
    ) -> None:
        """
        Initialize the detector configuration.

        Args:
            adaptiveThreshConstant (int): The constant used in adaptive thresholding.
            adaptiveThreshWinSizeMin (int): The minimum window size for adaptive thresholding.
            adaptiveThreshWinSizeMax (int): The maximum window size for adaptive thresholding.
            adaptiveThreshWinSizeStep (int): The step size for the window size in adaptive thresholding.
            minMarkerPerimeterRate (float): The minimum marker perimeter rate.
            maxMarkerPerimeterRate (float): The maximum marker perimeter rate.
            polygonalApproxAccuracyRate (float): The accuracy rate for polygonal approximation.
            minCornerDistanceRate (float): The minimum corner distance rate.
            minDistanceToBorder (int): The minimum distance to the border.
            minMarkerDistanceRate (float): The minimum marker distance rate.
            markerBorderBits (int): The number of bits in the marker border.
            perspectiveRemovePixelPerCell (int): The number of pixels per cell for perspective removal.
            perspectiveRemoveIgnoredMarginPerCell (float): The ignored margin per cell for perspective removal.
            maxErroneousBitsInBorderRate (float): The maximum erroneous bits in the border rate.
            minOtsuStdDev (float): The minimum Otsu standard deviation.
            cornerRefinementMethod (int): The method used for corner refinement.
            cornerRefinementWinSize (int): The window size for corner refinement.
            cornerRefinementMaxIterations (float): The maximum iterations for corner refinement.
            cornerRefinementMinAccuracy (float): The minimum accuracy for corner refinement.
            errorCorrectionRate (float): The error correction rate.
            useAruco3Detection (int): Whether to use ArUco3 detection or not.
            minSideLengthCanonicalImg (int): The minimum side length of the canonical image.
            minMarkerLengthRatioOriginalImg (float): The minimum marker length ratio in the original image.
        """
        self.adaptiveThreshConstant = adaptiveThreshConstant
        self.adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin
        self.adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax
        self.adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep
        self.minMarkerPerimeterRate = minMarkerPerimeterRate
        self.maxMarkerPerimeterRate = maxMarkerPerimeterRate
        self.polygonalApproxAccuracyRate = polygonalApproxAccuracyRate
        self.minCornerDistanceRate = minCornerDistanceRate
        self.minDistanceToBorder = minDistanceToBorder
        self.minMarkerDistanceRate = minMarkerDistanceRate
        self.markerBorderBits = markerBorderBits
        self.perspectiveRemovePixelPerCell = perspectiveRemovePixelPerCell
        self.perspectiveRemoveIgnoredMarginPerCell = (
            perspectiveRemoveIgnoredMarginPerCell
        )
        self.maxErroneousBitsInBorderRate = maxErroneousBitsInBorderRate
        self.minOtsuStdDev = minOtsuStdDev
        self.cornerRefinementMethod = cornerRefinementMethod
        self.cornerRefinementWinSize = cornerRefinementWinSize
        self.cornerRefinementMaxIterations = cornerRefinementMaxIterations
        self.cornerRefinementMinAccuracy = cornerRefinementMinAccuracy
        self.errorCorrectionRate = errorCorrectionRate
        self.useAruco3Detection = useAruco3Detection
        self.minSideLengthCanonicalImg = minSideLengthCanonicalImg
        self.minMarkerLengthRatioOriginalImg = minMarkerLengthRatioOriginalImg

    def read_config_file(self, file_path: str, root_dir: str = ".") -> None:
        """
        Read the camera configuration from a yaml file.

        Args:
            file_path (str): The path to the yaml configuration file.
            root_dir (str): The root directory to resolve relative paths.
        """
        with open(os.path.join(root_dir, file_path), "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

            for key, value in config.items():
                if hasattr(self, key):
                    setattr(self, key, value)

    def set(self, name: str, value) -> None:
        """
        Set an attribute of the motor configuration.

        Args:
            attr_name (str): The name of the attribute to set.
            value: The value to set for the attribute.
        """
        
        if hasattr(self, name):
            setattr(self, name, value)
        else:
            raise AttributeError(f"MotorConfig has no attribute '{name}'")