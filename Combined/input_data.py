# ============================================================
# input_data.py
# All test instances for the 2D Cutting Stock Problem.
# No optimization code here.
# ============================================================

INSTANCES = {

    1: {
        "description": "Original 5-item instance",
        "W": 60,
        "H": 30,
        "items_data": {
            'A': {'demand': 160, 'w': 3, 'h': 2},
            'B': {'demand': 150, 'w': 5, 'h': 3},
            'C': {'demand': 170, 'w': 2, 'h': 2},
            'D': {'demand': 130, 'w': 6, 'h': 4},
            'E': {'demand': 120, 'w': 7, 'h': 5},
        },
    },

    2: {
        "description": "Small 3-item easy instance",
        "W": 20,
        "H": 10,
        "items_data": {
            'A': {'demand': 50, 'w': 3, 'h': 2},
            'B': {'demand': 40, 'w': 4, 'h': 3},
            'C': {'demand': 60, 'w': 2, 'h': 2},
        },
    },

    3: {
        "description": "Medium 4-item instance",
        "W": 40,
        "H": 20,
        "items_data": {
            'A': {'demand': 80, 'w': 4, 'h': 3},
            'B': {'demand': 70, 'w': 6, 'h': 4},
            'C': {'demand': 90, 'w': 3, 'h': 2},
            'D': {'demand': 60, 'w': 8, 'h': 5},
        },
    },

    4: {
        "description": "Tight sheet - large items",
        "W": 30,
        "H": 20,
        "items_data": {
            'A': {'demand': 40, 'w': 8,  'h': 6},
            'B': {'demand': 35, 'w': 10, 'h': 7},
            'C': {'demand': 50, 'w': 6,  'h': 4},
        },
    },

    5: {
        "description": "Wide sheet, many small items",
        "W": 100,
        "H": 20,
        "items_data": {
            'A': {'demand': 200, 'w': 2, 'h': 2},
            'B': {'demand': 150, 'w': 3, 'h': 2},
            'C': {'demand': 100, 'w': 4, 'h': 3},
            'D': {'demand': 80,  'w': 5, 'h': 4},
        },
    },

    6: {
        "description": "6-item mixed sizes",
        "W": 50,
        "H": 30,
        "items_data": {
            'A': {'demand': 100, 'w': 3, 'h': 2},
            'B': {'demand': 90,  'w': 5, 'h': 4},
            'C': {'demand': 110, 'w': 4, 'h': 3},
            'D': {'demand': 70,  'w': 7, 'h': 5},
            'E': {'demand': 80,  'w': 6, 'h': 4},
            'F': {'demand': 60,  'w': 8, 'h': 6},
        },
    },

    7: {
        "description": "Square sheet, uniform demand",
        "W": 40,
        "H": 40,
        "items_data": {
            'A': {'demand': 100, 'w': 5, 'h': 5},
            'B': {'demand': 100, 'w': 8, 'h': 4},
            'C': {'demand': 100, 'w': 4, 'h': 8},
            'D': {'demand': 100, 'w': 6, 'h': 6},
        },
    },

    8: {
        "description": "High demand stress test",
        "W": 60,
        "H": 40,
        "items_data": {
            'A': {'demand': 300, 'w': 3, 'h': 2},
            'B': {'demand': 250, 'w': 5, 'h': 3},
            'C': {'demand': 280, 'w': 4, 'h': 3},
            'D': {'demand': 200, 'w': 6, 'h': 4},
            'E': {'demand': 180, 'w': 7, 'h': 5},
        },
    },

    9: {
        "description": "Low demand, small sheet",
        "W": 15,
        "H": 10,
        "items_data": {
            'A': {'demand': 20, 'w': 2, 'h': 2},
            'B': {'demand': 15, 'w': 3, 'h': 2},
            'C': {'demand': 10, 'w': 4, 'h': 3},
        },
    },

    10: {
        "description": "7-item large benchmark",
        "W": 80,
        "H": 50,
        "items_data": {
            'A': {'demand': 200, 'w': 3, 'h': 2},
            'B': {'demand': 180, 'w': 5, 'h': 3},
            'C': {'demand': 220, 'w': 4, 'h': 3},
            'D': {'demand': 160, 'w': 6, 'h': 4},
            'E': {'demand': 140, 'w': 7, 'h': 5},
            'F': {'demand': 120, 'w': 8, 'h': 6},
            'G': {'demand': 100, 'w': 9, 'h': 7},
        },
    },
}
