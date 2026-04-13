
import yt_dlp

url = "https://youtube.com/clip/UgkxWSeFPW7OE3zTC9PaErJxu2zgu7jdjamD?si=LJIUOuqjv3wa2miX"

ydl_opts = {
    'format': 'bestvideo[height<=720]+bestaudio/best[height<=720]',  # limit to 720p
    'merge_output_format': 'mp4',  # ensure final file is mp4
    'outtmpl': '%(title)s.%(ext)s'  # save as video title.mp4
}

with yt_dlp.YoutubeDL(ydl_opts) as ydl:
    ydl.download([url])


# Run this too
# ffmpeg -i input.mp4 -c:v libx264 -crf 23 -c:a copy output.mp4