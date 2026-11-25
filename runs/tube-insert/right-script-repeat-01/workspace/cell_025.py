try:
 review('Calibrated top shows rack translated slightly and upright tube guided into second opening, with depth not directly visible.',poses={rack:prop(rt2,'z','xy')}); jaw(.036,True)
except Exception as e: print(type(e),str(e))