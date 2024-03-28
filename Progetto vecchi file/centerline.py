import centerline_width
centerline_width.extractPointsToTextFile(left_kml="Yas_Marina.kml",
					text_output_name="Yas_Marina.txt")
centerline_width.convertColumnsToCSV(text_file="Yas_Marina.txt")


river_object = centerline_width.riverCenterline(csv_data="Yas_Marina.csv")

river_object.plotCenterline()