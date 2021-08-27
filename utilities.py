def cm_to_pixel(cm, map_shape=800, physical_shape=200):
    """
    Converts cm to pixel. Expects a square shaped map
    :param cm: Input cm
    :param map_shape: Shape of the map (1-axis) in pixels. Preferred to be a multiple of 200 for round numbers.
    :param physical_shape: Shape of the map (1-axis) in real life in cm. Expected 200
    :return: Number of pixels to represent the cm. Will round off floats.
    """

    return round((map_shape / physical_shape) * cm)


if __name__ == "__main__":
    print(cm_to_pixel(50))
