import React from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import { useLocation } from '@docusaurus/router';
import TranslateButton from '@site/src/components/TranslateButton';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  const location = useLocation();

  // Only show translate button on week pages (chapter content)
  const isChapterPage = /\/week-\d+/.test(location.pathname);

  return (
    <>
      {isChapterPage && <TranslateButton />}
      <Content {...props} />
    </>
  );
}
