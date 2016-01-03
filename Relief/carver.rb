#!/usr/bin/env ruby

require 'chunky_png'
require 'ruby-progressbar'
require 'pp'

# Which relief are we carving?
#$Relief = 'sf'
#$Relief = 'yosemite'
#$Relief = 'sunnyside'
$Relief = 'halfdome'
#$Relief = 'devils'
#$Relief = 'sf400'
#$Relief = 'twinpeaks'

# Input filename
$ImageFilename = "#{$Relief}.png"

# Output filename prefix
$GCodeFilenamePrefix = "#{$Relief}-gcode"

# All measurements in inches, unless specified otherwise.

# How far the block starts from the origin
$OffsetX     = 1.00
$OffsetY     = 1.00

# Dimensions of the block
$BlockX      = 5.25
$BlockY      = 5.25
$BlockZ      = 1.5

# How the relief gets carved
#$ReliefMinZ  = 5.0 / 8.0
$ReliefMinZ  = 0.5
$ReliefMaxZ  = $BlockZ - 1.00 / 16.00

# Height at which to taxi around
$TaxiZ       = $BlockZ + 1.00 / 8.00

# Tool dimensions
# COARSE pass tool
$CoarseRadius   = 1.00 / 8.00
$CoarseCutHeight= 1.00
# FINE pass tool
$FineRadius     = 1.00 / 32.00
$FineCutHeight  = 3.00 / 8.00
$ShankRadius    = 1.00 / 16.00

# The distance between every move on Y axis
$CarveRes = ( $FineRadius ) / 4.00

# The space between the coarse and fine pass to ensure the fine tool
# will have material to shave off everywhere
$Shaving = 1.0 / 64.0

# How deep the tool can be to carve out material (how much material
# can be carved at a given depth) - determines how many times we have
# to pass over a spot to reach maximum penetration depth...
$CarveDepth  = 1.00

# Corresponds to a G0 movement (not linear) at maximum (accelerated) speed
$MaxSpeed    = 0.0
# How fast the tool moves during carving and while taxiing
$CoarseSpeed = 10.0
$FineSpeed = 60.0
$TaxiSpeed   = $MaxSpeed


# Direction of cutting of the X- and Y-axis - if the tool does not
# 'bite' into the material, you need to reverse one of these
# directions. Note this is only for the "coarse" pass. Fine goes both ways.
$ReverseX    = false
$ReverseY    = true

# Flipping the image, if it comes out mirrored, or upside down
$FlipX       = true
$FlipY       = false


#-------

# some range checks...
raise "Depest cut must be smaller than $CoarseCutHeight!" if $BlockZ - $ReliefMinZ > $CoarseCutHeight 
raise "$ReliefMinZ must be smaller than $ReliefMaxZ!" if $ReliefMinZ >= $ReliefMaxZ
raise "$BlockX should be a tad bigger (for numerical stability)!" if $BlockX < 0.01
raise "$BlockY should be a tad bigger (for numerical stability)!" if $BlockY < 0.01
raise "Highest relief is higher than block!" if $BlockZ < $ReliefMaxZ

#-------

def pixelValue(c)
  ChunkyPNG::Color.r(c)
end

STDERR.puts "Loading image...#{$ImageFilename}"
$png = ChunkyPNG::Image.from_file($ImageFilename)
$ImageX = $png.width.to_f
$ImageY = $png.height.to_f
STDERR.puts "Loaded map with X: #{$png.width} and Y: #{$png.height}"

$minRed = 255
$maxRed = 0
$png.pixels.each do |c|
  red = pixelValue(c)
  if red < $minRed then $minRed = red end
  if red > $maxRed then $maxRed = red end
end
$rangeRed = $maxRed - $minRed

STDERR.puts "Range of red channel: #{$minRed}..#{$maxRed} (#{$rangeRed} levels)"

block_aspect_ratio = $BlockX / $BlockY
image_aspect_ratio = $ImageX / $ImageY
if image_aspect_ratio < block_aspect_ratio then
  $aspect = $ImageX / $BlockX
else
  $aspect = $ImageY / $BlockY
end

$BlockXPixels = $BlockX * $aspect
$BlockYPixels = $BlockY * $aspect
$ImageOffsetX = (($ImageX - $BlockXPixels) / 2).to_i
$ImageOffsetY = (($ImageY - $BlockYPixels) / 2).to_i

STDERR.puts "Block Aspect Ratio: #{block_aspect_ratio}"
STDERR.puts "Image Aspect Ratio: #{image_aspect_ratio}"
STDERR.puts "Inch2PixelCoefficient: #{$aspect}"
STDERR.puts "Pixels on block: #{$BlockXPixels}  #{$BlockYPixels}"
STDERR.puts "Skipped Pixels: #{$ImageOffsetX} #{$ImageOffsetY}"
STDERR.puts "One pixel of the source image corresponds to #{1.0 / $aspect} inches."
STDERR.puts "Slicing the image in increments of #{$CarveRes} inches."

$prevX = nil
$prevY = nil
$prevZ = nil
$prevS = nil

def goto(file, x, y, z, s)
  x += $OffsetX
  y += $OffsetY

  line = []

  if s != $prevS then
    if s == $MaxSpeed then
      line << "G0"
    else
      if( $prevS == $MaxSpeed ) then line << "G1" end
      line << "F#{s}"
    end
    $prevS = s
  end

  if x != $prevX then
    line << "X" + '%.5f' % x
    $prevX = x
  end

  if y != $prevY then
    line << "Y" + '%.5f' % y
    $prevY = y
  end

  if z != $prevZ then
    line << "Z" + '%.5f' % z
    $prevZ = z
  end

  file.puts line.join(' ') unless line.empty?
end

def maxHeight(x, y, carveRadius)
  x = $BlockX - x if $FlipX
  y = $BlockY - y if $FlipY
  cx = ($ImageOffsetX + x * $aspect).to_i
  cy = ($ImageOffsetY + y * $aspect).to_i
  radius = (carveRadius * $aspect).ceil
  squared_radius = radius * radius
  range = (-radius..radius)
  result = $minRed
  range.each do |xr|
    range.each do |yr|
      if (xr * xr + yr * yr) < squared_radius then
        # inside round tool, now make sure it's inside the image for the lookup
        x = cx + xr
        y = cy + yr
        if (0 <= x) and (x < $ImageX) and (0 <= y) and (y < $ImageY) then
          red = pixelValue($png[x, y])
          if red > result then result = red end
        end
      end
    end
  end
  result
end

# This returns the difference between the brightest and darkest pixel in a region
def deltaHeight( x1, y1, x2, y2 )
  # Translate the physical location to image pixel coordinates
  if( $FlipX ) then
    px1 = ($ImageOffsetX + ( $BlockX - x2 ) * $aspect).to_i
    px2 = ($ImageOffsetX + ( $BlockX - x1 ) * $aspect).to_i
  else
    px1 = ($ImageOffsetX + x1 * $aspect).to_i
    px2 = ($ImageOffsetX + x2 * $aspect).to_i
  end
  if( $FlipY ) then
    py1 = ($ImageOffsetY + ( $BlockX - y2 ) * $aspect).to_i
    py2 = ($ImageOffsetY + ( $BlockX - y1 ) * $aspect).to_i
  else
    py1 = ($ImageOffsetY + y1 * $aspect).to_i
    py2 = ($ImageOffsetY + y2 * $aspect).to_i
  end

  # Check we're inside the image
  if( px2 >= $ImageX ) then px2 = $ImageX - 1 end
  if( py2 >= $ImageY ) then py2 = $ImageY - 1 end
  if( px1 < 0 ) then px1 = 0 end
  if( py1 < 0 ) then py1 = 0 end

  min = $maxRed
  max = $minRed
  rangeX = (px1..px2)
  rangeY = (py1..py2)
  rangeX.each do |x|
    rangeY.each do |y|
      red = pixelValue($png[x, y])
      if red > max then max = red end
      if red < min then min = red end
    end
  end
  max - min
end

###########
# SCANNING
###########

# This is the distance from one side of the fine cutting tip to the edge of the shank
$ToolStepSize = $ShankRadius + $FineRadius
# This is the dimension in red color range of the fine tool cutting length
$MaxDelta = ($FineCutHeight * $rangeRed / ($ReliefMaxZ - $ReliefMinZ)).to_i
# Scan the image in squares of $ToolStepSize. Make then overlap by so that we dont
# miss any steep area at the edge of 2 squares
$ScanStep = 2

STDERR.puts "Looking for relief steeper than #{$MaxDelta} red / #{$ToolStepSize} inches."

totalSteps = (($ScanStep * $BlockX / $ToolStepSize) * ($ScanStep * $BlockY / $ToolStepSize)).to_i
progress = ProgressBar.create(:format => '%e |%b>>%i| %P%% %c/%C %t',
                              :title => "Scanning",
                              :total => totalSteps)

# Start with the max we can cut with the desired relief shape. Anything below this, we can do
maxDelta = $MaxDelta
badX = 0
badY = 0

# Optimization: First two loops look at square area as potential problem area. If the difference 
# between the lowest and the highest pixes in this area is more than the current max, then it needs
# to be scanned with the actual round. The tool shape for all the positions within this area to find
# out if there is really a problem.
sqX = 0
while sqX<$BlockX-$ToolStepSize do
  sqY = 0
  while sqY<$BlockY-$ToolStepSize do
    if( deltaHeight( sqX, sqY, sqX + $ToolStepSize, sqY + $ToolStepSize ) > maxDelta ) then
      carveX = sqX
      while carveX < sqX + $ToolStepSize do
        carveY = sqY
        while carveY < sqY + $ToolStepSize do
          delta = maxHeight(carveX, carveY, $ShankRadius) - maxHeight(carveX, carveY, $FineRadius)         
          if( delta > $MaxDelta ) then 
            maxDelta = delta
            badX = carveX
            badY = carveY
          end
          carveY += $CarveRes
        end
        carveX += $CarveRes
      end
    end
    sqY += $ToolStepSize / $ScanStep
    progress.increment if progress.progress < (totalSteps - 1)
  end
  sqX += $ToolStepSize / $ScanStep
end

progress.finish 

# We found an area which is too steep to be cut. Rise the bottom of the relief
# as much as needed to reduce the slope so that it's doable.
if( maxDelta - $MaxDelta > 0 ) then
  optimalMinZ = $ReliefMaxZ - ($FineCutHeight * $rangeRed / maxDelta)
  STDERR.puts "CarveError is #{maxDelta-$MaxDelta} at location #{badX},#{badY} inches."
  STDERR.puts "ReliefMinZ has been raised from #{$ReliefMinZ} to #{optimalMinZ} inches."
else
  # No problem, keep desired relief bottom level
  optimalMinZ = $ReliefMinZ
end

##############
# COARSE pass
##############
carveStepX = [1.0 / $aspect, $CoarseRadius].max
carveStepY = [1.0 / $aspect, $CarveRes].max
stepsX = ($BlockX / carveStepX).ceil
stepsY = ($BlockY / carveStepY).ceil
totalSteps = stepsX * stepsY
$prevS = -1

progress = ProgressBar.create(:format => '%e |%b>>%i| %P%% %c/%C %t',
                              :title => "Coarse pass.",
                              :total => totalSteps)


File.open("#{$GCodeFilenamePrefix}-coarse.txt", "w") do |file|
  file.write "G10 G40 G90 M3 "
  goto file, -$OffsetX, -$OffsetY, $TaxiZ, $TaxiSpeed
  x = 0
  while x < stepsX do
    carveX = ($ReverseX ? ((stepsX - 1) - x) : x) * carveStepX
    y = 0
    carveY = ($ReverseY ? ((stepsY - 1) - y) : y) * carveStepY
    goto file, carveX, carveY, $TaxiZ, $TaxiSpeed
    prevZ = 0.0
    prevY = 0.0
    while y < stepsY do

      redValue = maxHeight(carveX, carveY, $CoarseRadius + $Shaving)
      reliefZ = $Shaving * 2 + optimalMinZ + (($ReliefMaxZ - optimalMinZ) * (redValue - $minRed)) / $rangeRed

      edge = (y == 0) || (y == (stepsY - 1))
      if( prevZ == reliefZ && !edge ) then
        prevY = carveY
      else
        if( prevY != 0.0 ) then goto file, carveX, prevY, prevZ, $CoarseSpeed end
        prevY = 0
        prevZ = reliefZ
        goto file, carveX, carveY, reliefZ, $CoarseSpeed       
      end

      y += 1
      carveY = ($ReverseY ? ((stepsY - 1) - y) : y) * carveStepY
      progress.increment if progress.progress < (totalSteps - 1)
    end

    goto file, carveX, carveY, $TaxiZ, $TaxiSpeed
    x += 1
  end
  progress.finish  
  # go back to rest position
  goto file, -$OffsetX, -$OffsetY, $TaxiZ, $TaxiSpeed
  file.write "M0 "
  goto file, -$OffsetX, -$OffsetY, 0.0, $TaxiSpeed
end

############
# FINE Pass
############
carveStepX = [1.0 / $aspect, $CarveRes].max
carveStepY = [1.0 / $aspect, $CarveRes].max
stepsX = ($BlockX / carveStepX).ceil
stepsY = ($BlockY / carveStepY).ceil
totalSteps = stepsX * stepsY
$prevS = -1

progress = ProgressBar.create(:format => '%e |%b>>%i| %P%% %c/%C %t',
                              :title => "Fine pass.",
                              :total => totalSteps)

File.open("#{$GCodeFilenamePrefix}-fine.txt", "w") do |file|
  file.write "G10 G40 G90 M3 "
  goto file, -$OffsetX, -$OffsetY, $TaxiZ, $TaxiSpeed

  x = 0
  y = 0
  carveX = ($ReverseX ? ((stepsX - 1) - x) : x) * carveStepX
  carveY = ($ReverseY ? ((stepsY - 1) - y) : y) * carveStepY
  goto file, carveX, carveY, $TaxiZ, $TaxiSpeed

  while x < stepsX do
    prevY = 0.0
    prevZ = 0.0

    while y < stepsY do
      redValue = maxHeight(carveX, carveY, $FineRadius)
      reliefZ = optimalMinZ + (($ReliefMaxZ - optimalMinZ) * (redValue - $minRed)) / $rangeRed

      edge = ( y < 5 || y > (stepsY - 6)) 
      if( edge ) then speed = $FineSpeed / 3 else speed = $FineSpeed end

      # If the relief altitude has not changed 
      if( reliefZ == prevZ && !edge ) then
        prevY = carveY
      else
        if( prevY != 0.0 ) then goto file, carveX, prevY, prevZ, speed end
        prevY = 0.0
        prevZ = reliefZ     
        goto file, carveX, carveY, reliefZ, speed
      end
      y += 1
      carveY = (($ReverseY ^ (x & 1 == 1)) ? ((stepsY - 1) - y) : y) * carveStepY
      progress.increment if progress.progress < (totalSteps - 1)
    end
    y = 0
    x += 1
    carveX = ($ReverseX ? ((stepsX - 1) - x) : x) * carveStepX
  end
  
  progress.finish  

  # go back to rest position
  goto file, carveX, carveY, $TaxiZ, $TaxiSpeed
  file.write "M0 "
  goto file, -$OffsetX, -$OffsetY, $TaxiZ, $TaxiSpeed
  goto file, -$OffsetX, -$OffsetY, 0.0, $TaxiSpeed

  STDERR.puts "Done."

end

