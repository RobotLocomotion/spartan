

class CategoryManipulationType(object):
    SHOE_ON_TABLE = 0
    MUG_ON_TABLE = 1
    MUG_ON_TABLE_ROTATION_INVARIANT = 2
    MUG_ON_TABLE_3_KEYPOINTS = 3
    MUG_ON_RACK = 4
    MUG_ON_SHELF_3D = 5
    SHOE_ON_RACK = 6

    @staticmethod
    def from_string(val):
        v = val.upper()
        if v == "SHOE_ON_TABLE":
            return CategoryManipulationType.SHOE_ON_TABLE
        elif v == "MUG_ON_TABLE":
            return CategoryManipulationType.MUG_ON_TABLE
        elif v == "MUG_ON_TABLE_ROTATION_INVARIANT":
            return CategoryManipulationType.MUG_ON_TABLE_ROTATION_INVARIANT
        elif v == "MUG_ON_TABLE_3_KEYPOINTS":
            return CategoryManipulationType.MUG_ON_TABLE_3_KEYPOINTS
        elif v == "MUG_ON_RACK":
            return CategoryManipulationType.MUG_ON_RACK
        elif v == "MUG_ON_SHELF_3D":
            return CategoryManipulationType.MUG_ON_SHELF_3D
        elif v == "SHOE_ON_RACK":
            return CategoryManipulationType.SHOE_ON_RACK
        else:
            raise ValueError("unknown category manipulation type")

    @staticmethod
    def to_string(v):

        if v ==  CategoryManipulationType.SHOE_ON_TABLE:
            return "SHOE_ON_TABLE"
        elif v == CategoryManipulationType.MUG_ON_TABLE:
            return "MUG_ON_TABLE"
        elif v == CategoryManipulationType.MUG_ON_TABLE_ROTATION_INVARIANT:
            return "MUG_ON_TABLE_ROTATION_INVARIANT"
        elif v == CategoryManipulationType.MUG_ON_TABLE_3_KEYPOINTS:
            return "MUG_ON_TABLE_3_KEYPOINTS"
        elif v == CategoryManipulationType.MUG_ON_RACK:
            return "MUG_ON_RACK"
        elif v == CategoryManipulationType.MUG_ON_SHELF_3D:
            return "MUG_ON_SHELF_3D"
        elif v == CategoryManipulationType.SHOE_ON_RACK:
            return "SHOE_ON_RACK"
        else:
            raise ValueError("unknown category manipulation type")