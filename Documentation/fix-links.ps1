param (
    [string]$sitePath = "./_site",
    [string]$repoUrl = "https://github.com/bepu/bepuphysics2/blob/master"
)

# Get all HTML files in the site, excluding those in the api/ directory
$htmlFiles = Get-ChildItem -Path $sitePath -Filter "*.html" -Recurse | Where-Object { $_.FullName -notmatch "\\api\\" }

$linkCount = 0
foreach ($file in $htmlFiles) {
    $content = Get-Content -Path $file.FullName -Raw
    $originalContent = $content
    
    # Find links that start with "../" and transform them to GitHub URLs
    $pattern = '(href=["''])(\.\.\/[^"'']*)(["''])'
    
    # Use a scriptblock for the replacement to keep the original quote style and only modify the URL
    $newContent = [regex]::Replace($content, $pattern, {
        param($match)
        $prefix = $match.Groups[1].Value  # href=" or href='
        $path = $match.Groups[2].Value    # ../path
        $suffix = $match.Groups[3].Value  # " or '
        
        # Remove the "../" prefix for the GitHub URL
        $relativePath = $path -replace '^\.\.\/', ''
        
        # Return the full replacement with the same quote style
        return "$prefix$repoUrl/$relativePath$suffix"
    })
    
    # Only write the file if changes were made
    if ($newContent -ne $originalContent) {
        $matches = [regex]::Matches($content, $pattern)
        $linkCount += $matches.Count
        Set-Content -Path $file.FullName -Value $newContent
        Write-Host "Fixed $($matches.Count) links in $($file.Name)"
    }
}

Write-Host "Link transformation complete. Fixed $linkCount links in total."